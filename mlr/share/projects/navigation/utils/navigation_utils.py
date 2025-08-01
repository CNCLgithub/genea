import crocoddyl
import numpy as np
import pinocchio

from example_robot_data.robots_loader import TalosLegsLoader


class NavAgent:
    TALOS_LEGS = "talos_legs"

    def __init__(self, agent_name):
        self._name = agent_name

        self._robot = None
        self._left_joint_name = None
        self._right_joint_name = None

        self._init_agent()

    def _init_agent(self):
        if self._name == NavAgent.TALOS_LEGS:
            self._robot = TalosLegsLoader().robot
            self._left_joint_name = "left_sole_link"
            self._right_joint_name = "right_sole_link"

    def get_agent(self):
        return self._robot

    def get_agent_model(self):
        return self.get_agent().model

    def get_agent_data(self):
        return self.get_agent().data

    def get_left_joint_name(self):
        return self._left_joint_name

    def get_right_joint_name(self):
        return self._right_joint_name

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

    def get_left_frame_id(self):
        return self.get_agent_model().getFrameId(self._left_joint_name)

    def get_right_frame_id(self):
        return self.get_agent_model().getFrameId(self._right_joint_name)

    def get_joint_pos(self, frame_id):
        return self.get_agent_data().oMf[frame_id].translation

    def get_left_joint_pos(self):
        return self.get_agent_data().oMf[self.get_left_frame_id()].translation

    def get_right_joint_pos(self):
        return self.get_agent_data().oMf[self.get_right_frame_id()].translation

    def get_joint_frame_id_list(self):
        return self.get_left_frame_id(), self.get_right_frame_id()

    def get_joint_pos_list(self):
        return [self.get_joint_pos(frame_id) for frame_id in self.get_joint_frame_id_list()]


class NavPosition:
    def __init__(self, x, y, z):
        self._pos_x = x
        self._pos_y = y
        self._pos_z = z

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
