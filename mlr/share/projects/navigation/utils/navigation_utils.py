import crocoddyl
import numpy as np
import pinocchio

from example_robot_data.robots_loader import TalosLegsLoader

from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg


class NavAgent:
    TALOS_LEGS = "talos_legs"

    def __init__(self, agent_name):
        self._name = agent_name

        self._robot = None
        self._left_foot_joint_name = None
        self._right_foot_joint_name = None

        self._init_agent()

    def _init_agent(self):
        if self._name == NavAgent.TALOS_LEGS:
            self._robot = TalosLegsLoader().robot
            self._left_foot_joint_name = "left_sole_link"
            self._right_foot_joint_name = "right_sole_link"

    def _get_left_foot_joint_name(self):
        return self._left_foot_joint_name

    def _get_right_foot_joint_name(self):
        return self._right_foot_joint_name

    def _get_joint_frame_id(self, joint_name):
        return self.get_agent_model().getFrameId(joint_name)

    def _get_joint_pos(self, joint_frame_id):
        return self.get_agent_data().oMf[joint_frame_id].translation

    def get_name(self):
        return self._name

    def get_agent(self):
        return self._robot

    def get_agent_model(self):
        return self.get_agent().model

    def get_agent_data(self):
        return self.get_agent().data

    def get_left_foot_frame_id(self):
        return self._get_joint_frame_id(self._get_left_foot_joint_name())

    def get_right_foot_frame_id(self):
        return self._get_joint_frame_id(self._get_right_foot_joint_name())

    def get_left_foot_pos(self):
        return self._get_joint_pos(self.get_left_foot_frame_id())

    def get_right_foot_pos(self):
        return self._get_joint_pos(self.get_right_foot_frame_id())

    def get_total_mass(self):
        return sum([inertial.mass for inertial in self.get_agent_model().inertias])

    def get_nv(self):
        return self.get_agent_model().nv

    def get_nq(self):
        return self.get_agent_model().nq

    def get_x0(self):
        q0 = self.get_q0()
        v0 = pinocchio.utils.zero(self.get_nv())
        return np.concatenate([q0, v0])

    def get_q0(self):
        return self.get_agent_model().referenceConfigurations["half_sitting"].copy()

    def get_state0(self):
        return np.concatenate([self.get_q0(), np.zeros(self.get_nv())])

    def get_state(self):
        return crocoddyl.StateMultibody(self.get_agent_model())

    def get_agent_actuation_model(self):
        return crocoddyl.ActuationModelFloatingBase(self.get_state())

    def get_nu(self):
        return self.get_agent_actuation_model().nu


class NavPosition:
    def __init__(self, x, y, z):
        self._pos_x = x
        self._pos_y = y
        self._pos_z = z

    def add_x(self, x):
        self._pos_x += x

    def set_x(self, new_x):
        self._pos_x = new_x

    def get_x(self):
        return self._pos_x

    def get_y(self):
        return self._pos_y

    def get_z(self):
        return self._pos_z

    def get_position_as_list(self):
        return [self._pos_x, self._pos_y, self._pos_z]

    def get_position_as_np_array(self):
        return np.array(self.get_position_as_list())

    def get_position_as_str(self):
        return f"{self._pos_x} {self._pos_y} {self._pos_z}"


class NavRotation:
    def __init__(self, roll, pitch, yaw):
        self._rot_roll = roll
        self._rot_pitch = pitch
        self._rot_yaw = yaw

    def get_rotation_as_list(self):
        return [self._rot_roll, self._rot_pitch, self._rot_yaw]

    def get_rotation_as_np_array(self):
        return np.array(self.get_rotation_as_list())

    def get_rotation_as_str(self):
        return f"{self._rot_roll} {self._rot_pitch} {self._rot_yaw}"


class NavPose:
    def __init__(self, position: NavPosition, rotation: NavRotation=NavRotation(0.0, 0.0, 0.0)):
        self._position = position
        self._rotation = rotation

    def get_position(self):
        return self._position

    def get_rotation(self):
        return self._rotation


class NavForce:
    def __init__(self, force_pose: NavPose, force_magnitude):
        self._force_magnitude = force_magnitude
        self._force_pose = force_pose

    def get_force_magnitude(self):
        return self._force_magnitude

    def get_force_pose(self):
        return self._force_pose


class NavTask:
    JUMP = "jump"
    WALK = "walk"

    def __init__(self, task_type):
        self._task_type = task_type
        self._task_solver = None

    def set_task_solver(self, task_solver):
        self._task_solver = task_solver

    def get_task_type(self):
        return self._task_type

    def get_task_problem(self, agent: NavAgent, current_pose):
        pass

    def get_task_solver(self):
        return self._task_solver

    def get_task_cost(self):
        return self._task_solver.cost


class _NavConstraint:
    def __init__(self, joint_id, joint_pos=None, joint_misc_data=None):
        self._joint_id = joint_id
        self._joint_pos = joint_pos
        self._joint_misc_data = joint_misc_data

    def get_joint_id(self):
        return self._joint_id

    def get_joint_pos(self):
        return self._joint_pos

    def get_joint_misc_data(self):
        return self._joint_misc_data


class NavProblemConstraints:
    def __init__(self, nav_agent: NavAgent):
        self._agent = nav_agent

        self._com_constraint = None

        self._contact_constraints_list = []
        self._swing_constraints_list = []

    def _add_joint_contact_constraint(self, joint_id):
        self._contact_constraints_list.append(_NavConstraint(joint_id=joint_id))

    def _add_swing_constraint(self, joint_id, joint_pos, disp_vector):
        if not isinstance(disp_vector, np.ndarray):
            Msg.print_error("ERROR [NavProblem]: displacement vector must be specified as an np array")
            assert False
        self._swing_constraints_list.append(_NavConstraint(joint_id, joint_pos, disp_vector))

    def add_l_contact_constraint(self):
        self._add_joint_contact_constraint(self._agent.get_left_foot_frame_id())

    def add_r_contact_constraint(self):
        self._add_joint_contact_constraint(self._agent.get_right_foot_frame_id())

    def add_l_swing_constraint(self, disp_vec):
        self._add_swing_constraint(self._agent.get_left_foot_frame_id(), self._agent.get_left_foot_pos(), disp_vec)

    def add_r_swing_constraint(self, disp_vec):
        self._add_swing_constraint(self._agent.get_right_foot_frame_id(), self._agent.get_right_foot_pos(), disp_vec)

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

    def get_swing_constraints_disp_vec_list(self):
        return [constraint.get_joint_misc_data() for constraint in self.get_swing_constraints()]


class NavProblem:
    def __init__(self, nav_agent: NavAgent):
        self._agent = nav_agent

    def create_foot_action_phase(self, nav_constraints):
        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), self._agent.get_nu())
        contacts_list = crocoddyl.ContactModelMultiple(self._agent.get_state(), self._agent.get_nu())

        contacts_frame_id_list = nav_constraints.get_contact_constraints_frame_id_list()

        swing_frame_id_list = nav_constraints.get_swing_constraints_frame_id_list()
        swing_pos_list = nav_constraints.get_swing_constraints_pos_list()
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
            wc = crocoddyl.WrenchCone(np.eye(3), ConfigUtils.NAV_PROBLEM_FRICTION_COEFFICIENT, np.array([0.1, 0.05]))
            wc_res = crocoddyl.ResidualModelContactWrenchCone(self._agent.get_state(), f_id, wc, self._agent.get_nu())
            wc_act = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(wc.lb, wc.ub))
            wc_cost = crocoddyl.CostModelResidual(self._agent.get_state(), wc_act, wc_res)
            costs_list.addCost(str(f_id) + "_wrench_cone", wc_cost, 1e1)

        for f_id, f_pos, f_disp_vec in zip(swing_frame_id_list, swing_pos_list, swing_disp_vec_list):
            pos = pinocchio.SE3(np.eye(3), f_pos + f_disp_vec)
            f_res = crocoddyl.ResidualModelFramePlacement(self._agent.get_state(), f_id, pos, self._agent.get_nu())
            f_cost = crocoddyl.CostModelResidual(self._agent.get_state(), f_res)
            costs_list.addCost(str(f_id) + "_foot_track", f_cost, 1e6)

        state_nv = self._agent.get_state().nv
        state_weights = np.array([0] * 3 + [500.0] * 3 + [0.01] * (state_nv - 6) + [10] * state_nv)
        s_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_state0(), self._agent.get_nu())
        s_act = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        s_cost = crocoddyl.CostModelResidual(self._agent.get_state(), s_act, s_res)
        costs_list.addCost("state_cost", s_cost, 1e1)

        ctrl_res = crocoddyl.ResidualModelControl(self._agent.get_state(), self._agent.get_nu())
        ctrl_cost = crocoddyl.CostModelResidual(self._agent.get_state(), ctrl_res)
        costs_list.addCost("ctrl_cost", ctrl_cost, 1e-1)

        diff_action_model = crocoddyl.DifferentialActionModelContactFwdDynamics(self._agent.get_state(),
                                                                                self._agent.get_agent_actuation_model(),
                                                                                contacts_list, costs_list, 0.0, True)

        control = crocoddyl.ControlParametrizationModelPolyZero(self._agent.get_nu())

        return crocoddyl.IntegratedActionModelEuler(diff_action_model, control, ConfigUtils.NAV_PROBLEM_TIME_STEP)

    def create_foot_impulse_phase(self, nav_constraints):
        foot_frame_id_list = nav_constraints.get_swing_constraints_frame_id_list()
        foot_pos_list = nav_constraints.get_swing_constraints_pos_list()
        foot_disp_vec_list = nav_constraints.get_swing_constraints_disp_vec_list()

        costs_list = crocoddyl.CostModelSum(self._agent.get_state(), 0)

        for f_id, f_pos, f_disp_vec in zip(foot_frame_id_list, foot_pos_list, foot_disp_vec_list):
            pos = pinocchio.SE3(np.eye(3), f_pos + f_disp_vec)
            fp_res = crocoddyl.ResidualModelFramePlacement(self._agent.get_state(), f_id, pos, 0)
            fp_cost = crocoddyl.CostModelResidual(self._agent.get_state(), fp_res)
            costs_list.addCost(str(f_id) + "_foot_track", fp_cost, 1e8)

        state_weights = np.array([1.0] * 6 + [0.1] * (self._agent.get_nv() - 6) + [10] * self._agent.get_nv())
        state_act = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        state_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_state0(), 0)
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
            wc = crocoddyl.WrenchCone(np.eye(3), ConfigUtils.NAV_PROBLEM_FRICTION_COEFFICIENT, np.array([0.1, 0.05]))
            wc_res = crocoddyl.ResidualModelContactWrenchCone(self._agent.get_state(), f_id, wc, self._agent.get_nu())
            wc_act = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(wc.lb, wc.ub))
            wc_cost = crocoddyl.CostModelResidual(self._agent.get_state(), wc_act, wc_res)
            costs_list.addCost(str(f_id) + "_wrench_cone", wc_cost, 1e1)

        for f_id, f_pos, f_disp_vec in zip(foot_frame_id_list, foot_pos_list, foot_disp_vec_list):
            pos = pinocchio.SE3(np.eye(3), f_pos + f_disp_vec)
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
        s_act = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        s_res = crocoddyl.ResidualModelState(self._agent.get_state(), self._agent.get_state0(), self._agent.get_nu())
        state_cost = crocoddyl.CostModelResidual(self._agent.get_state(), s_act, s_res)
        costs_list.addCost("state_cost", state_cost, 1e1)

        ctrl_res = crocoddyl.ResidualModelControl(self._agent.get_state(), self._agent.get_nu())
        ctrl_cost = crocoddyl.CostModelResidual(self._agent.get_state(), ctrl_res)
        costs_list.addCost("ctrl_cost", ctrl_cost, 1e-3)

        diff_action_model = crocoddyl.DifferentialActionModelContactFwdDynamics(self._agent.get_state(),
                                                                                self._agent.get_agent_actuation_model(),
                                                                                contacts_list, costs_list, 0.0, True)

        return crocoddyl.IntegratedActionModelEuler(diff_action_model, 0.0)
