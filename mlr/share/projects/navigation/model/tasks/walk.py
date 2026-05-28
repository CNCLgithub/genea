import crocoddyl
import pinocchio

import numpy as np
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils

from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavAgent, NavProblem, NavProblemConstraints


class WalkTask(NavTask):
    def __init__(self, is_first_step=False):
        super().__init__(NavTask.WALK)

        self._is_first_step = is_first_step
        self._step_length = 0.0
        self._step_height = 0.0

    def set_step_length(self, step_length: float):
        self._step_length = step_length

    def set_step_height(self, step_height: float):
        self._step_height = step_height

    def is_first_step(self):
        return self._is_first_step

    def get_step_length(self):
        return self._step_length

    def get_step_height(self):
        return self._step_height

    def get_task_problem(self, agent: NavAgent, current_pose):
        p = WalkProblem(agent)
        return p.create_walk_problem(current_pose, self.get_step_length(), self.get_step_height(), self.is_first_step())


class WalkProblem(NavProblem):
    def __init__(self, nav_agent: NavAgent):
        super().__init__(nav_agent)

    @staticmethod
    def _get_disp_vec(step_index, step_length, step_height, total_steps):
        x_value = step_index * step_length / total_steps

        z_increment = step_height / (total_steps // 2)
        if step_index <= total_steps // 2:
            z_value = step_index * z_increment
        else:
            z_value = (total_steps - step_index) * z_increment

        return np.array([x_value, 0.0, z_value])

    def create_walk_problem(self, current_pose, step_length, step_height, is_first_step):
        pose0 = current_pose[: self._agent.get_nq()]
        pinocchio.forwardKinematics(self._agent.get_agent_model(), self._agent.get_agent_data(), pose0)
        pinocchio.updateFramePlacements(self._agent.get_agent_model(), self._agent.get_agent_data())

        com_ref = (self._agent.get_left_foot_pos() + self._agent.get_right_foot_pos()) / 2
        com_ref[2] = pinocchio.centerOfMass(self._agent.get_agent_model(), self._agent.get_agent_data(), pose0)[2]

        total_steps = ConfigUtils.WALK_TREAD_KNOTS

        walk_problem_list = []

        walk_stand_phase = []
        for _ in range(ConfigUtils.WALK_STAND_KNOTS):
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_contact_constraint()
            walk_stand_phase.append(self.create_foot_action_phase(nav_constraint))

        right_step_len = step_length
        if is_first_step:
            right_step_len = step_length * 0.5

        walk_tread_right_phase = []
        for step_index in range(1, total_steps + 1):
            com_vec = np.array([step_index * right_step_len / total_steps, 0.0, 0.0]) * 0.5 + com_ref
            disp_vec = self._get_disp_vec(step_index, right_step_len, step_height, total_steps)
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_swing_constraint(disp_vec)
            nav_constraint.add_com_constraint(com_vec)
            walk_tread_right_phase.append(self.create_foot_action_phase(nav_constraint))

        disp_vec = self._get_disp_vec(total_steps, right_step_len, step_height, total_steps)
        nav_constraint = NavProblemConstraints(self._agent)
        nav_constraint.add_r_swing_constraint(disp_vec)
        walk_tread_right_phase.append(self.create_foot_impulse_phase(nav_constraint))

        com_ref[0] += step_length * 0.5

        walk_tread_left_phase = []
        for step_index in range(1, total_steps + 1):
            com_vec = np.array([step_index * step_length / total_steps, 0.0, 0.0]) * 0.5 + com_ref
            disp_vec = self._get_disp_vec(step_index, step_length, step_height, total_steps)
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_r_contact_constraint()
            nav_constraint.add_l_swing_constraint(disp_vec)
            nav_constraint.add_com_constraint(com_vec)
            walk_tread_left_phase.append(self.create_foot_action_phase(nav_constraint))

        disp_vec = self._get_disp_vec(total_steps, step_length, step_height, total_steps)
        nav_constraint = NavProblemConstraints(self._agent)
        nav_constraint.add_l_swing_constraint(disp_vec)
        walk_tread_left_phase.append(self.create_foot_impulse_phase(nav_constraint))

        walk_problem_list += walk_stand_phase + walk_tread_right_phase
        walk_problem_list += walk_stand_phase + walk_tread_left_phase

        return crocoddyl.ShootingProblem(current_pose, walk_problem_list[:-1], walk_problem_list[-1])
