import crocoddyl
import pinocchio
import numpy as np

from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.config_utils import NavConfig
from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavProblem, NavProblemConstraints, \
    NavTaskRegistry


class WalkTask(NavTask):
    def __init__(self, step_length=None, is_first=False, is_close=False, is_lunge=False):
        super().__init__(NavTask.WALK)

        self._step_length = step_length

        self._is_first_step = is_first
        self._is_close_step = is_close
        self._is_lunge_step = is_lunge

    def is_first_step(self):
        return self._is_first_step

    def is_close_step(self):
        return self._is_close_step

    def is_lunge_step(self):
        return self._is_lunge_step

    def step(self):
        return self._step_length is not None

    def tally_task(self, *platform_names_list):
        phase1 = NavConfig.WALK_STAND_KNOTS
        phase2 = NavConfig.WALK_TREAD_KNOTS + 1 + phase1
        phase3 = NavConfig.WALK_STAND_KNOTS + phase2
        phase4 = NavConfig.WALK_TREAD_KNOTS + 1 + phase3

        task_forces_by_time_list = self.get_task_forces_by_time_list()
        for time_index, task_forces_list in enumerate(task_forces_by_time_list):
            task_registry = NavTaskRegistry()

            if 0 <= time_index < phase1:
                task_registry.set_force_left(task_forces_list[0])
                task_registry.set_force_right(task_forces_list[1])
            elif phase1 <= time_index < phase2:
                task_registry.set_force_left(task_forces_list[0])
            elif phase2 <= time_index < phase3:
                task_registry.set_force_left(task_forces_list[0])
                task_registry.set_force_right(task_forces_list[1])
            elif phase3 <= time_index < phase4:
                task_registry.set_force_right(task_forces_list[0])

            if self.is_lunge_step():
                if 0 <= time_index < phase1:
                    task_registry.set_platform_name_left(platform_names_list[0])
                    task_registry.set_platform_name_right(platform_names_list[0])
                elif phase1 <= time_index < phase2:
                    task_registry.set_platform_name_left(platform_names_list[0])
                elif phase2 <= time_index < phase3:
                    task_registry.set_platform_name_left(platform_names_list[0])
                    task_registry.set_platform_name_right(platform_names_list[1])
                elif phase3 <= time_index < phase4:
                    task_registry.set_platform_name_right(platform_names_list[1])
            else:
                task_registry.set_platform_name_left(platform_names_list[0])
                task_registry.set_platform_name_right(platform_names_list[0])

            self.add_to_registry(task_registry)

    def get_task_problem(self, nav_agent: NavAgent):
        return WalkProblem(nav_agent).create_problem(self)

    def get_step_length(self):
        return self._step_length


class WalkProblem(NavProblem):
    def __init__(self, nav_agent: NavAgent):
        super().__init__(nav_agent)

    @staticmethod
    def _step_delta(step_index, step_length):
        return step_index * step_length / NavConfig.WALK_TREAD_KNOTS

    @staticmethod
    def _get_disp_vec(step_index, step_length, walk_vec):
        disp_vec = walk_vec * WalkProblem._step_delta(step_index, step_length)

        delta_z = NavConfig.MAX_STEP_HEIGHT / (NavConfig.WALK_TREAD_KNOTS // 2)
        if step_index <= NavConfig.WALK_TREAD_KNOTS // 2:
            disp_vec_z = step_index * delta_z
        else:
            disp_vec_z = (NavConfig.WALK_TREAD_KNOTS - step_index) * delta_z

        return np.array([disp_vec[0], disp_vec[1], disp_vec_z])

    def _get_r_constraint(self, disp_vec, com_vec=None):
        nav_constraint = NavProblemConstraints(self._agent)
        if com_vec is not None:
            nav_constraint.add_l_contact_constraint()
        nav_constraint.add_r_swing_constraint(disp_vec)
        if com_vec is not None:
            nav_constraint.add_com_constraint(com_vec)
        return nav_constraint

    def _get_l_constraint(self, disp_vec, com_vec=None):
        nav_constraint = NavProblemConstraints(self._agent)
        if com_vec is not None:
            nav_constraint.add_r_contact_constraint()
        nav_constraint.add_l_swing_constraint(disp_vec)
        if com_vec is not None:
            nav_constraint.add_com_constraint(com_vec)
        return nav_constraint

    def get_tread_actions_list(self, step_length, walk_vec, com_ref, is_left_step=False):
        tread_actions_list = []

        for step_index in range(1, NavConfig.WALK_TREAD_KNOTS + 1):
            com_vec = walk_vec * WalkProblem._step_delta(step_index, step_length) * 0.5 + com_ref
            disp_vec = self._get_disp_vec(step_index, step_length, walk_vec)
            if is_left_step:
                nav_constraint = self._get_l_constraint(disp_vec, com_vec)
            else:
                nav_constraint = self._get_r_constraint(disp_vec, com_vec)
            tread_actions_list.append(self.create_foot_action_phase(nav_constraint))

        disp_vec = self._get_disp_vec(NavConfig.WALK_TREAD_KNOTS, step_length, walk_vec)
        if is_left_step:
            nav_constraint = self._get_l_constraint(disp_vec)
        else:
            nav_constraint = self._get_r_constraint(disp_vec)
        tread_actions_list.append(self.create_foot_impulse_phase(nav_constraint))

        return tread_actions_list

    def get_stand_actions_list(self):
        stand_actions_list = []
        for _ in range(NavConfig.WALK_STAND_KNOTS):
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_contact_constraint()
            stand_actions_list.append(self.create_foot_action_phase(nav_constraint))
        return stand_actions_list

    def create_problem(self, walk_task: WalkTask):
        step_length = walk_task.get_step_length()

        pose = self.get_agent_pose()[: self._agent.get_nq()]
        pinocchio.forwardKinematics(self._agent.get_agent_model(), self._agent.get_agent_data(), pose)
        pinocchio.updateFramePlacements(self._agent.get_agent_model(), self._agent.get_agent_data())

        com_ref = (self._agent.get_left_foot_pos() + self._agent.get_right_foot_pos()) / 2
        com_ref[2] = pinocchio.centerOfMass(self._agent.get_agent_model(), self._agent.get_agent_data(), pose)[2]

        walk_vec = pinocchio.Quaternion(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix()[:, 0].copy()  # noqa
        walk_vec[2] = 0.0
        walk_vec /= (np.linalg.norm(walk_vec) + 1e-12)

        r_step_length = step_length
        l_step_length = step_length
        if walk_task.is_first_step():
            r_step_length = step_length * 0.5
        if walk_task.is_close_step():
            l_step_length = step_length * 0.5

        problem_list = []
        problem_list += self.get_stand_actions_list()
        problem_list += self.get_tread_actions_list(r_step_length, walk_vec, com_ref, is_left_step=False)

        com_ref += walk_vec * step_length * 0.5

        problem_list += self.get_stand_actions_list()
        problem_list += self.get_tread_actions_list(l_step_length, walk_vec, com_ref, is_left_step=True)

        return crocoddyl.ShootingProblem(self.get_agent_pose(), problem_list[:-1], problem_list[-1])
