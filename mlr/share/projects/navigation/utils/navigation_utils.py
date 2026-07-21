import crocoddyl
import numpy as np
import pinocchio

from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import NavConfig
from mlr.share.projects.navigation.utils.core_utils import NavForce, NavPose, NavPosition
from mlr.share.projects.navigation.utils.crocoddyl_utils import CrocoddylUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg


class NavTaskRegistry:
    def __init__(self):
        self._nav_force_left = None
        self._nav_force_right = None

        self._nav_platform_name_left = None
        self._nav_platform_name_right = None

    def set_force_left(self, nav_force: NavForce):
        self._nav_force_left = nav_force

    def set_force_right(self, nav_force: NavForce):
        self._nav_force_right = nav_force

    def set_platform_name_left(self, platform_name: str):
        self._nav_platform_name_left = platform_name

    def set_platform_name_right(self, platform_name: str):
        self._nav_platform_name_right = platform_name

    def get_force_left(self) -> NavForce:
        return self._nav_force_left

    def get_force_left_pos(self) -> np.ndarray:
        return self.get_force_left().get_force_pose().get_position().get_position_as_np_array()

    def get_force_left_vec(self) -> np.ndarray:
        force_mag = self.get_force_left().get_force_magnitude()
        return self.get_force_left().get_force_pose().get_rotation().get_rotation_as_np_array() * force_mag

    def get_force_right(self) -> NavForce:
        return self._nav_force_right

    def get_force_right_pos(self) -> np.ndarray:
        return self.get_force_right().get_force_pose().get_position().get_position_as_np_array()

    def get_force_right_vec(self) -> np.ndarray:
        force_mag = self.get_force_right().get_force_magnitude()
        return self.get_force_right().get_force_pose().get_rotation().get_rotation_as_np_array() * force_mag

    def get_platform_name_left(self) -> str:
        return self._nav_platform_name_left

    def get_platform_name_right(self) -> str:
        return self._nav_platform_name_right


class NavTask:
    JUMP = "jump"
    WALK = "walk"
    TURN = "turn"

    def __init__(self, task_type):
        self._task_type = task_type
        self._task_solver = None
        self._task_registry_list = []

    @staticmethod
    def validate_force(nav_force, cutoff):
        if nav_force.get_force_norm() > cutoff:
            nav_force.set_force_magnitude(nav_force.get_force_magnitude() * 10. ** -1)

    @staticmethod
    def get_random_force(force_pos_vec):
        roll = ComputeUtils.sample_uniform(-100, 100).item()
        pitch = ComputeUtils.sample_uniform(-100, 100).item()
        yaw = ComputeUtils.sample_uniform(-1000, -2500).item()

        force_pose = NavPose(NavPosition(force_pos_vec[0], force_pos_vec[1], force_pos_vec[2]))
        force_pose.set_rpy(roll, pitch, yaw)

        return NavForce(force_pose, 0.001)

    def add_to_registry(self, nav_task_registry: NavTaskRegistry):
        self._task_registry_list.append(nav_task_registry)

    def run_task(self, nav_agent: NavAgent):
        self._task_solver = crocoddyl.SolverFDDP(self.get_task_problem(nav_agent))
        self._task_solver.th_stop = NavConfig.DYNAMICS_THRESHOLD

        if NavConfig.DEBUG_KINEMATICS:
            self._task_solver.setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackLogger()])  # noqa

        _xs = [nav_agent.get_current_pose()] * (self._task_solver.problem.T + 1)
        _us = self._task_solver.problem.quasiStatic([nav_agent.get_current_pose()] * self._task_solver.problem.T)
        self._task_solver.solve(_xs, _us, 100, False)

        nav_agent.update_current_pose(self._task_solver.xs[-1])

    def tally_task(self, *platform_names_list):
        pass

    def get_task_problem(self, agent: NavAgent):
        pass

    def get_task_type(self):
        return self._task_type

    def get_task_solver(self):
        return self._task_solver

    def get_task_cost_crocoddyl(self):
        if self._task_solver is None:
            return 0.0
        return sum(CrocoddylUtils.get_costs_list(self._task_solver))

    def get_task_cost_ke(self):
        if self._task_solver is None:
            return 0.0

        pin_model = self.get_task_solver().problem.runningModels[0].state.pinocchio
        pin_data = pin_model.createData()

        nq = pin_model.nq
        nv = pin_model.nv

        total_ke = 0.0
        for pose_vec in self.get_task_solver().xs:
            q = pose_vec[:nq]
            v = pose_vec[nq:nq + nv]
            total_ke += pinocchio.computeKineticEnergy(pin_model, pin_data, q, v)
        return total_ke

    def get_task_forces_by_time_list(self):
        return CrocoddylUtils.get_forces_by_time_list(self._task_solver)

    def get_task_registry_list(self):
        return self._task_registry_list


class _NavConstraint:
    def __init__(self, joint_id, joint_pos=None, joint_rot=None, joint_disp_vec=None):
        self._joint_id = joint_id
        self._joint_pos = joint_pos
        self._joint_rot = joint_rot
        self._joint_disp_vec = joint_disp_vec

    def get_joint_id(self):
        return self._joint_id

    def get_joint_pos(self):
        return self._joint_pos

    def get_joint_rot(self):
        return self._joint_rot

    def get_joint_disp_vec(self):
        return self._joint_disp_vec


class NavProblemConstraints:
    def __init__(self, nav_agent: NavAgent):
        self._agent = nav_agent

        self._com_constraint = None

        self._contact_constraints_list = []
        self._swing_constraints_list = []

    def _add_joint_contact_constraint(self, joint_id):
        self._contact_constraints_list.append(_NavConstraint(joint_id=joint_id))

    def _add_swing_constraint(self, joint_id, joint_pos, joint_rot, disp_vec):
        if not isinstance(disp_vec, np.ndarray):
            Msg.print_error("ERROR [NavProblem]: displacement vector must be specified as an np array")
            assert False
        self._swing_constraints_list.append(_NavConstraint(joint_id, joint_pos, joint_rot, disp_vec))

    def add_l_contact_constraint(self):
        self._add_joint_contact_constraint(self._agent.get_left_foot_frame_id())

    def add_r_contact_constraint(self):
        self._add_joint_contact_constraint(self._agent.get_right_foot_frame_id())

    def add_l_swing_constraint(self, disp_vec):
        self._add_swing_constraint(self._agent.get_left_foot_frame_id(),
                                   self._agent.get_left_foot_pos(),
                                   self._agent.get_left_foot_rot(),
                                   disp_vec)

    def add_r_swing_constraint(self, disp_vec):
        self._add_swing_constraint(self._agent.get_right_foot_frame_id(),
                                   self._agent.get_right_foot_pos(),
                                   self._agent.get_right_foot_rot(),
                                   disp_vec)

    def add_com_constraint(self, com_vec):
        self._com_constraint = com_vec

    def get_contact_constraints(self):
        return self._contact_constraints_list

    def get_swing_constraints(self):
        return self._swing_constraints_list

    def get_com_constraint(self):
        return self._com_constraint

    def get_contact_constraints_frame_id_list(self):
        return [constraint.get_joint_id() for constraint in self.get_contact_constraints()]

    def get_swing_constraints_frame_id_list(self):
        return [constraint.get_joint_id() for constraint in self.get_swing_constraints()]

    def get_swing_constraints_pos_list(self):
        return [constraint.get_joint_pos() for constraint in self.get_swing_constraints()]

    def get_swing_constraints_rot_list(self):
        return [constraint.get_joint_rot() for constraint in self.get_swing_constraints()]

    def get_swing_constraints_disp_vec_list(self):
        return [constraint.get_joint_disp_vec() for constraint in self.get_swing_constraints()]


class NavProblem:
    def __init__(self, nav_agent: NavAgent):
        self._agent = nav_agent

    def create_problem(self, nav_task: NavTask):
        pass

    def create_foot_action_phase(self, nav_constraints):
        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), self._agent.get_nu())
        contacts_list = crocoddyl.ContactModelMultiple(self._agent.get_state(), self._agent.get_nu())

        contacts_frame_id_list = nav_constraints.get_contact_constraints_frame_id_list()

        swing_frame_id_list = nav_constraints.get_swing_constraints_frame_id_list()
        swing_pos_list = nav_constraints.get_swing_constraints_pos_list()
        swing_rot_list = nav_constraints.get_swing_constraints_rot_list()
        swing_disp_vec_list = nav_constraints.get_swing_constraints_disp_vec_list()

        com_vec = nav_constraints.get_com_constraint()
        if isinstance(com_vec, np.ndarray):
            com_res = crocoddyl.ResidualModelCoMPosition(self._agent.get_state(), com_vec, self._agent.get_nu())
            com_cost = crocoddyl.CostModelResidual(self._agent.get_state(), com_res)
            costs_list.addCost("com_track", com_cost, 1e6)

        for f_id in contacts_frame_id_list:
            contact_model = crocoddyl.ContactModel6D(self._agent.get_state(),
                                                     f_id,
                                                     pinocchio.SE3.Identity(),  # noqa
                                                     pinocchio.LOCAL_WORLD_ALIGNED,
                                                     self._agent.get_nu(),
                                                     np.array([0.0, 30.0]))
            contacts_list.addContact(str(f_id) + "_contact", contact_model)

        for f_id in contacts_frame_id_list:
            wc = crocoddyl.WrenchCone(np.eye(3), NavConfig.FRICTION_COEFFICIENT, np.array([0.1, 0.05]))
            wc_res = crocoddyl.ResidualModelContactWrenchCone(self._agent.get_state(), f_id, wc, self._agent.get_nu())
            wc_act = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(wc.lb, wc.ub))
            wc_cost = crocoddyl.CostModelResidual(self._agent.get_state(), wc_act, wc_res)
            costs_list.addCost(str(f_id) + "_wrench_cone", wc_cost, 1e1)

        for f_id, f_pos, f_rot, f_vec in zip(swing_frame_id_list, swing_pos_list, swing_rot_list, swing_disp_vec_list):
            pos = pinocchio.SE3(f_rot, f_pos + f_vec)
            f_res = crocoddyl.ResidualModelFramePlacement(self._agent.get_state(), f_id, pos, self._agent.get_nu())
            f_cost = crocoddyl.CostModelResidual(self._agent.get_state(), f_res)
            costs_list.addCost(str(f_id) + "_foot_track", f_cost, 1e6)

        state_nv = self._agent.get_state().nv
        state_weights = np.array([0] * 3 + [500.0] * 3 + [0.01] * (state_nv - 6) + [10] * state_nv)
        s_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_ref_pose(), self._agent.get_nu())
        s_act = crocoddyl.ActivationModelWeightedQuad(state_weights ** 2)
        s_cost = crocoddyl.CostModelResidual(self._agent.get_state(), s_act, s_res)
        costs_list.addCost("state_cost", s_cost, 1e1)

        ctrl_res = crocoddyl.ResidualModelControl(self._agent.get_state(), self._agent.get_nu())
        ctrl_cost = crocoddyl.CostModelResidual(self._agent.get_state(), ctrl_res)
        costs_list.addCost("ctrl_cost", ctrl_cost, 1e-1)

        diff_action_model = crocoddyl.DifferentialActionModelContactFwdDynamics(self._agent.get_state(),
                                                                                self._agent.get_agent_actuation_model(),
                                                                                contacts_list, costs_list, 0.0, True)

        control = crocoddyl.ControlParametrizationModelPolyZero(self._agent.get_nu())

        return crocoddyl.IntegratedActionModelEuler(diff_action_model, control, NavConfig.TIME_STEP)

    def create_foot_impulse_phase(self, nav_constraints):
        foot_frame_id_list = nav_constraints.get_swing_constraints_frame_id_list()
        foot_pos_list = nav_constraints.get_swing_constraints_pos_list()
        foot_rot_list = nav_constraints.get_swing_constraints_rot_list()
        foot_disp_vec_list = nav_constraints.get_swing_constraints_disp_vec_list()

        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), 0)

        for f_id, f_pos, f_rot, f_vec in zip(foot_frame_id_list, foot_pos_list, foot_rot_list, foot_disp_vec_list):
            pos = pinocchio.SE3(f_rot, f_pos + f_vec)
            fp_res = crocoddyl.ResidualModelFramePlacement(self._agent.get_state(), f_id, pos, 0)
            fp_cost = crocoddyl.CostModelResidual(self._agent.get_state(), fp_res)
            costs_list.addCost(str(f_id) + "_foot_track", fp_cost, 1e8)

        state_weights = np.array([1.0] * 6 + [0.1] * (self._agent.get_nv() - 6) + [10] * self._agent.get_nv())
        state_act = crocoddyl.ActivationModelWeightedQuad(state_weights ** 2)
        state_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_ref_pose(), 0)
        state_cost = crocoddyl.CostModelResidual(self._agent.get_state(), state_act, state_res)
        costs_list.addCost("state_cost", state_cost, 1e1)

        impulses_list = crocoddyl.ImpulseModelMultiple(self._agent.get_state())
        for f_id in foot_frame_id_list:
            contact_model = crocoddyl.ImpulseModel6D(self._agent.get_state(), f_id, pinocchio.LOCAL_WORLD_ALIGNED)
            impulses_list.addImpulse(str(f_id) + "_impulse", contact_model)

        impulse_model = crocoddyl.ActionModelImpulseFwdDynamics(self._agent.get_state(), impulses_list, costs_list)
        impulse_model.JMinvJt_damping = 1e-12
        impulse_model.r_coeff = 0.0

        return impulse_model

    def create_foot_pseudo_impulse_phase(self, nav_constraints):
        foot_contacts_frame_id_list = nav_constraints.get_contact_constraints_frame_id_list()

        foot_frame_id_list = nav_constraints.get_swing_constraints_frame_id_list()
        foot_pos_list = nav_constraints.get_swing_constraints_pos_list()
        foot_rot_list = nav_constraints.get_swing_constraints_rot_list()
        foot_disp_vec_list = nav_constraints.get_swing_constraints_disp_vec_list()

        contacts_list = crocoddyl.ContactModelMultiple(self._agent.get_state(), self._agent.get_nu())
        for f_id in foot_contacts_frame_id_list:
            contact_model = crocoddyl.ContactModel6D(self._agent.get_state(),
                                                     f_id,
                                                     pinocchio.SE3.Identity(),  # noqa
                                                     pinocchio.LOCAL_WORLD_ALIGNED,
                                                     self._agent.get_nu(),
                                                     np.array([0.0, 50.0]))
            contacts_list.addContact(str(f_id) + "_contact", contact_model)

        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), self._agent.get_nu())
        for f_id in foot_contacts_frame_id_list:
            wc = crocoddyl.WrenchCone(np.eye(3), NavConfig.TIME_STEP, np.array([0.1, 0.05]))
            wc_res = crocoddyl.ResidualModelContactWrenchCone(self._agent.get_state(), f_id, wc, self._agent.get_nu())
            wc_act = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(wc.lb, wc.ub))
            wc_cost = crocoddyl.CostModelResidual(self._agent.get_state(), wc_act, wc_res)
            costs_list.addCost(str(f_id) + "_wrench_cone", wc_cost, 1e1)

        for f_id, f_pos, f_rot, f_vec in zip(foot_frame_id_list, foot_pos_list, foot_rot_list, foot_disp_vec_list):
            pos = pinocchio.SE3(f_rot, f_pos + f_vec)
            f_res = crocoddyl.ResidualModelFramePlacement(self._agent.get_state(), f_id, pos, self._agent.get_nu())
            fv_res = crocoddyl.ResidualModelFrameVelocity(self._agent.get_state(), f_id,
                                                          pinocchio.Motion.Zero(),
                                                          pinocchio.LOCAL_WORLD_ALIGNED,
                                                          self._agent.get_nu())
            f_cost = crocoddyl.CostModelResidual(self._agent.get_state(), f_res)
            fv_cost = crocoddyl.CostModelResidual(self._agent.get_state(), fv_res)
            costs_list.addCost(str(f_id) + "_foot_track", f_cost, 1e8)
            costs_list.addCost(str(f_id) + "_foot_impulse", fv_cost, 1e6)

        state_nv = self._agent.get_state().nv
        state_weights = np.array([0] * 3 + [500.0] * 3 + [0.01] * (state_nv - 6) + [10] * state_nv)
        s_act = crocoddyl.ActivationModelWeightedQuad(state_weights ** 2)
        s_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_ref_pose(), self._agent.get_nu())
        state_cost = crocoddyl.CostModelResidual(self._agent.get_state(), s_act, s_res)
        costs_list.addCost("state_cost", state_cost, 1e1)

        ctrl_res = crocoddyl.ResidualModelControl(self._agent.get_state(), self._agent.get_nu())
        ctrl_cost = crocoddyl.CostModelResidual(self._agent.get_state(), ctrl_res)
        costs_list.addCost("ctrl_cost", ctrl_cost, 1e-3)

        diff_action_model = crocoddyl.DifferentialActionModelContactFwdDynamics(self._agent.get_state(),
                                                                                self._agent.get_agent_actuation_model(),
                                                                                contacts_list, costs_list, 0.0, True)

        return crocoddyl.IntegratedActionModelEuler(diff_action_model, 0.0)

    def get_agent_pose(self):
        return self._agent.get_current_pose()
