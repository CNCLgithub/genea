import crocoddyl
import pinocchio

from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.config_utils import NavConfig
from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavProblem, NavProblemConstraints, \
    NavTaskRegistry


class JumpTask(NavTask):
    def __init__(self, jump_vector):
        super().__init__(NavTask.JUMP)

        self._jump_vector = jump_vector

    def tally_task(self, *platform_names_list):
        phase1 = NavConfig.JUMP_GROUND_KNOTS - 1
        phase2 = NavConfig.JUMP_FLYING_KNOTS + phase1 + 1
        phase3 = NavConfig.JUMP_FLYING_KNOTS + 1 + phase2
        phase4 = NavConfig.JUMP_GROUND_KNOTS + phase3

        task_forces_by_time_list = self.get_task_forces_by_time_list()
        for time_index, task_forces_list in enumerate(task_forces_by_time_list):
            task_registry = NavTaskRegistry()

            if len(task_forces_list) > 0:
                JumpTask.validate_force(task_forces_list[0])
            if len(task_forces_list) > 1:
                JumpTask.validate_force(task_forces_list[1])

            if phase3 <= time_index < phase4:
                task_registry.set_force_left(task_forces_list[0])
                task_registry.set_force_right(task_forces_list[1])
                task_registry.set_platform_name_left(platform_names_list[1])
                task_registry.set_platform_name_right(platform_names_list[1])

            self.add_to_registry(task_registry)

    def get_jump_vector(self):
        return self._jump_vector

    def get_task_problem(self, nav_agent: NavAgent):
        return JumpProblem(nav_agent).create_problem(self)


class JumpProblem(NavProblem):
    def __init__(self, nav_agent: NavAgent):
        super().__init__(nav_agent)
    
    def create_problem(self, jump_task: JumpTask):
        pose = self.get_agent_pose()[: self._agent.get_nq()]
        pinocchio.forwardKinematics(self._agent.get_agent_model(), self._agent.get_agent_data(), pose)
        pinocchio.updateFramePlacements(self._agent.get_agent_model(), self._agent.get_agent_data())
        
        height_diff = jump_task.get_jump_vector()[2] - self._agent.get_right_foot_pos()[2]

        com_ref = (self._agent.get_left_foot_pos() + self._agent.get_right_foot_pos()) / 2
        com_ref[2] = pinocchio.centerOfMass(self._agent.get_agent_model(), self._agent.get_agent_data(), pose)[2]

        com_jump = jump_task.get_jump_vector() / 2.0
        com_jump[2] += NavConfig.MAX_JUMP_HEIGHT

        take_off_phase = []
        for _ in range(NavConfig.JUMP_GROUND_KNOTS):
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_contact_constraint()
            take_off_phase.append(self.create_foot_action_phase(nav_constraint))

        fly_up_phase = []
        for k in range(NavConfig.JUMP_FLYING_KNOTS):
            com_jump_segment = com_jump * (k + 1) / NavConfig.JUMP_FLYING_KNOTS + com_ref
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_com_constraint(com_jump_segment)
            fly_up_phase.append(self.create_foot_action_phase(nav_constraint))

        fly_down_phase = []
        for k in range(NavConfig.JUMP_FLYING_KNOTS):
            nav_constraint = NavProblemConstraints(self._agent)
            fly_down_phase.append(self.create_foot_action_phase(nav_constraint))

        nav_constraint = NavProblemConstraints(self._agent)
        nav_constraint.add_l_swing_constraint(jump_task.get_jump_vector())
        nav_constraint.add_r_swing_constraint(jump_task.get_jump_vector())
        touch_down_phase = [self.create_foot_impulse_phase(nav_constraint)]

        land_phase = []
        land_vec = jump_task.get_jump_vector().copy()
        land_vec[2] = height_diff
        for _ in range(NavConfig.JUMP_GROUND_KNOTS // 2):
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_contact_constraint()
            nav_constraint.add_com_constraint(com_ref + land_vec)
            land_phase.append(self.create_foot_action_phase(nav_constraint))

        problem_list = []
        problem_list += take_off_phase
        problem_list += fly_up_phase
        problem_list += fly_down_phase
        problem_list += touch_down_phase
        problem_list += land_phase

        return crocoddyl.ShootingProblem(self.get_agent_pose(), problem_list[:-1], problem_list[-1])
