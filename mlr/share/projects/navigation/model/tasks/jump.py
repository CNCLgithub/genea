import crocoddyl
import numpy as np
import pinocchio

from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavAgent, NavProblem, NavProblemConstraints


class JumpTask(NavTask):
    def __init__(self):
        super().__init__(NavTask.JUMP)

        self._jump_vector = np.array([0.0, 0.0, 0.0])
        self._jump_height = 0.0

    def set_jump_height(self, jump_height):
        self._jump_height = jump_height

    def set_jump_vector(self, jump_vector):
        if not isinstance(jump_vector, np.ndarray):
            jump_vector = np.array(jump_vector)
        if jump_vector.shape != (3,):
            raise ValueError("ERROR [JumpTask]: the jump vector must be a 3D vector!")

        self._jump_vector = jump_vector

    def get_jump_vector(self):
        return self._jump_vector

    def get_jump_height(self):
        return self._jump_height

    def get_task_problem(self, agent: NavAgent, current_pose):
        jump_problem = JumpProblem(agent)
        return jump_problem.create_jump_problem(current_pose, self.get_jump_vector(), self.get_jump_height())


class JumpProblem(NavProblem):
    def __init__(self, nav_agent: NavAgent):
        super().__init__(nav_agent)
    
    def create_jump_problem(self, current_pose, jump_vector, jump_height):
        pose0 = current_pose[: self._agent.get_nq()]
        pinocchio.forwardKinematics(self._agent.get_agent_model(), self._agent.get_agent_data(), pose0)
        pinocchio.updateFramePlacements(self._agent.get_agent_model(), self._agent.get_agent_data())
        
        height_diff = jump_vector[2] - self._agent.get_right_foot_pos()[2]

        com_ref = (self._agent.get_left_foot_pos() + self._agent.get_right_foot_pos()) / 2
        com_ref[2] = pinocchio.centerOfMass(self._agent.get_agent_model(), self._agent.get_agent_data(), pose0)[2]

        com_jump = np.array([jump_vector[0] / 2.0,
                             jump_vector[1] / 2.0,
                             jump_vector[2] / 2.0 + jump_height])

        jump_problem_list = []

        take_off_phase = []
        for _ in range(ConfigUtils.JUMP_GROUND_KNOTS):
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_contact_constraint()
            take_off_phase.append(self.create_foot_action_phase(nav_constraint))

        fly_up_phase = []
        for k in range(ConfigUtils.JUMP_FLYING_KNOTS):
            com_jump_segment = com_jump * (k + 1) / ConfigUtils.JUMP_FLYING_KNOTS + com_ref
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_com_constraint(com_jump_segment)
            fly_up_phase.append(self.create_foot_action_phase(nav_constraint))

        fly_down_phase = []
        for k in range(ConfigUtils.JUMP_FLYING_KNOTS):
            nav_constraint = NavProblemConstraints(self._agent)
            fly_down_phase.append(self.create_foot_action_phase(nav_constraint))

        nav_constraint = NavProblemConstraints(self._agent)
        nav_constraint.add_l_swing_constraint(jump_vector)
        nav_constraint.add_r_swing_constraint(jump_vector)
        touch_down_phase = [self.create_foot_impulse_phase(nav_constraint)]

        land_phase = []
        land_vec = jump_vector.copy()
        land_vec[2] = height_diff
        for _ in range(ConfigUtils.JUMP_GROUND_KNOTS // 2):
            nav_constraint = NavProblemConstraints(self._agent)
            nav_constraint.add_l_contact_constraint()
            nav_constraint.add_r_contact_constraint()
            nav_constraint.add_com_constraint(com_ref + land_vec)
            land_phase.append(self.create_foot_action_phase(nav_constraint))

        jump_problem_list += take_off_phase
        jump_problem_list += fly_up_phase
        jump_problem_list += fly_down_phase
        jump_problem_list += touch_down_phase
        jump_problem_list += land_phase

        return crocoddyl.ShootingProblem(current_pose, jump_problem_list[:-1], jump_problem_list[-1])
