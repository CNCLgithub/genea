import crocoddyl
import numpy as np
import pinocchio

from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavAgent


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
        jump_problem = GeneaJumpProblem(agent)
        return jump_problem.create_jump_problem(current_pose, self.get_jump_vector(), self.get_jump_height())


class GeneaJumpProblem:
    TIME_STEP = 0.02
    GROUND_KNOTS = 20
    FLYING_KNOTS = 20

    FRICTION_COEFFICIENT = 0.7

    def __init__(self, nav_agent: NavAgent):
        self._agent = nav_agent
    
    def create_jump_problem(self, current_pose, jump_vector, jump_height):
        pose0 = current_pose[: self._agent.get_nq()]
        pinocchio.forwardKinematics(self._agent.get_agent_model(), self._agent.get_agent_data(), pose0)
        pinocchio.updateFramePlacements(self._agent.get_agent_model(), self._agent.get_agent_data())
        
        com_ref = (self._agent.get_left_joint_pos() + self._agent.get_right_joint_pos()) / 2
        height_diff = jump_vector[2] - self._agent.get_right_joint_pos()[2]

        com_ref[2] = pinocchio.centerOfMass(self._agent.get_agent_model(), self._agent.get_agent_data(), pose0)[2]
        com_jump = np.array([jump_vector[0] / 2.0,
                             jump_vector[1] / 2.0,
                             jump_vector[2] / 2.0 + jump_height])

        jump_problem_list = []

        take_off_phase = []
        for _ in range(GeneaJumpProblem.GROUND_KNOTS):
            take_off_phase.append(self.get_swing_foot_model())

        fly_up_phase = []
        for k in range(GeneaJumpProblem.FLYING_KNOTS):
            com_jump_segment = com_jump * (k + 1) / GeneaJumpProblem.FLYING_KNOTS + com_ref
            fly_up_phase.append(self.get_swing_foot_model(add_contact=False, com_pos=com_jump_segment))

        fly_down_phase = []
        for k in range(GeneaJumpProblem.FLYING_KNOTS):
            fly_down_phase.append(self.get_swing_foot_model(add_contact=False))

        touch_down_phase = [self.get_impulse_model(jump_vector)]

        land_phase = []
        land_vec = jump_vector.copy()
        land_vec[2] = height_diff
        for _ in range(GeneaJumpProblem.GROUND_KNOTS // 2):
            land_phase.append(self.get_swing_foot_model(com_pos=com_ref + land_vec))

        jump_problem_list += take_off_phase
        jump_problem_list += fly_up_phase
        jump_problem_list += fly_down_phase
        jump_problem_list += touch_down_phase
        jump_problem_list += land_phase

        return crocoddyl.ShootingProblem(current_pose, jump_problem_list[:-1], jump_problem_list[-1])

    def get_swing_foot_model(self, add_contact=True, com_pos=None):
        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), self._agent.get_nu())

        if isinstance(com_pos, np.ndarray):
            com_residual = crocoddyl.ResidualModelCoMPosition(self._agent.get_state(), com_pos, self._agent.get_nu())
            com_cost = crocoddyl.CostModelResidual(self._agent.get_state(), com_residual)
            costs_list.addCost("com_track", com_cost, 1e6)

        # define the friction cones for the feet contacts
        for f_id in self._agent.get_joint_frame_id_list():
            if not add_contact:
                continue
            wc = crocoddyl.WrenchCone(np.eye(3), GeneaJumpProblem.FRICTION_COEFFICIENT, np.array([0.1, 0.05]))
            wc_res = crocoddyl.ResidualModelContactWrenchCone(self._agent.get_state(), f_id, wc, self._agent.get_nu())
            wc_act = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(wc.lb, wc.ub))
            wc_cost = crocoddyl.CostModelResidual(self._agent.get_state(), wc_act, wc_res)
            costs_list.addCost(str(f_id) + "_wrench_cone", wc_cost, 1e1)

        state_nv = self._agent.get_state().nv
        state_weights = np.array([0] * 3 + [500.0] * 3 + [0.01] * (state_nv - 6) + [10] * state_nv)
        s_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_state0(), self._agent.get_nu())
        s_act = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        s_cost = crocoddyl.CostModelResidual(self._agent.get_state(), s_act, s_res)
        costs_list.addCost("state_cost", s_cost, 1e1)

        ctrl_residual = crocoddyl.ResidualModelControl(self._agent.get_state(), self._agent.get_nu())
        ctrl_cost = crocoddyl.CostModelResidual(self._agent.get_state(), ctrl_residual)
        costs_list.addCost("ctrl_cost", ctrl_cost, 1e-1)

        contacts_list = crocoddyl.ContactModelMultiple(self._agent.get_state(), self._agent.get_nu())
        for f_id in self._agent.get_joint_frame_id_list():
            if not add_contact:
                continue
            contact_model = crocoddyl.ContactModel6D(self._agent.get_state(),
                                                     f_id,
                                                     pinocchio.SE3.Identity(),  # noqa
                                                     pinocchio.LOCAL_WORLD_ALIGNED,
                                                     self._agent.get_nu(),
                                                     np.array([0.0, 30.0]))
            contacts_list.addContact(str(f_id) + "_contact", contact_model)

        diff_action_model = crocoddyl.DifferentialActionModelContactFwdDynamics(self._agent.get_state(),
                                                                                self._agent.get_agent_actuation_model(),
                                                                                contacts_list, costs_list, 0.0, True)

        control = crocoddyl.ControlParametrizationModelPolyZero(self._agent.get_nu())

        return crocoddyl.IntegratedActionModelEuler(diff_action_model, control, GeneaJumpProblem.TIME_STEP)

    def get_impulse_model(self, jump_vector):
        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), 0)

        for foot_id, foot_pos0 in zip(self._agent.get_joint_frame_id_list(), self._agent.get_joint_pos_list()):
            pos = pinocchio.SE3(np.eye(3), foot_pos0 + jump_vector)
            fp_residual = crocoddyl.ResidualModelFramePlacement(self._agent.get_state(), foot_id, pos, 0)
            fp_cost = crocoddyl.CostModelResidual(self._agent.get_state(), fp_residual)
            costs_list.addCost(str(foot_id) + "_foot_track", fp_cost, 1e8)

        state_weights = np.array([1.0] * 6 + [0.1] * (self._agent.get_nv() - 6) + [10] * self._agent.get_nv())
        state_activation = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        state_residual = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_state0(), 0)
        state_cost = crocoddyl.CostModelResidual(self._agent.get_state(), state_activation, state_residual)
        costs_list.addCost("state_cost", state_cost, 1e1)

        impulses_list = crocoddyl.ImpulseModelMultiple(self._agent.get_state())
        for foot_id in self._agent.get_joint_frame_id_list():
            contact_model = crocoddyl.ImpulseModel6D(self._agent.get_state(), foot_id, pinocchio.LOCAL_WORLD_ALIGNED)
            impulses_list.addImpulse(str(foot_id) + "_impulse", contact_model)

        impulse_model = crocoddyl.ActionModelImpulseFwdDynamics(self._agent.get_state(), impulses_list, costs_list)
        impulse_model.JMinvJt_damping = 1e-12
        impulse_model.r_coeff = 0.0

        return impulse_model
