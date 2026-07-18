import crocoddyl
import numpy as np
import pinocchio

from example_robot_data.talos import TalosLegsLoader

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils


class NavAgent:
    TALOS_LEGS = "talos_legs"

    def __init__(self, agent_name):
        self._name = agent_name

        self._robot = None
        self._left_foot_joint_name = None
        self._right_foot_joint_name = None

        self._init_agent()
        self._reference_pose = self.get_neutral_pose()
        self._current_pose = self.get_neutral_pose()
        self._current_pose[0] = ComputeUtils.sample_skew_normal(0., 0.15, -1, -0.5, 0.0).item()
        self._current_pose[1] = ComputeUtils.sample_uniform(-0.5, 0.5).item()

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

    def _get_joint_rot(self, joint_frame_id):
        return self.get_agent_data().oMf[joint_frame_id].rotation

    @staticmethod
    def rotate_to(agent_x0, target_angle_rad):
        new_pose = pinocchio.XYZQUATToSE3(agent_x0[:7])
        new_pose.rotation = pinocchio.utils.rotate("z", target_angle_rad)
        agent_x0[:7] = pinocchio.SE3ToXYZQUAT(new_pose)
        return agent_x0

    @staticmethod
    def get_robot():
        return TalosLegsLoader().robot

    def set_ref_pose(self, reference_pose):
        self._reference_pose = reference_pose

    def update_current_pose(self, current_pose):
        self._current_pose = current_pose

    def get_name(self):
        return self._name

    def get_agent_model(self):
        return self._robot.model

    def get_agent_data(self):
        return self._robot.data

    def get_left_foot_frame_id(self):
        return self._get_joint_frame_id(self._get_left_foot_joint_name())

    def get_right_foot_frame_id(self):
        return self._get_joint_frame_id(self._get_right_foot_joint_name())

    def get_left_foot_pos(self):
        return self._get_joint_pos(self.get_left_foot_frame_id())

    def get_left_foot_rot(self):
        return self._get_joint_rot(self.get_left_foot_frame_id())

    def get_right_foot_pos(self):
        return self._get_joint_pos(self.get_right_foot_frame_id())

    def get_right_foot_rot(self):
        return self._get_joint_rot(self.get_right_foot_frame_id())

    def get_total_mass(self):
        return sum([inertial.mass for inertial in self.get_agent_model().inertias])

    def get_nv(self):
        return self.get_agent_model().nv

    def get_nq(self):
        return self.get_agent_model().nq

    def get_neutral_pose(self):
        q0 = self.get_q0()
        v0 = pinocchio.utils.zero(self.get_nv())
        return np.concatenate([q0, v0])

    def get_q0(self):
        return self.get_agent_model().referenceConfigurations["half_sitting"].copy()

    def get_ref_pose(self):
        return self._reference_pose

    def get_current_pose(self):
        return self._current_pose

    def get_state(self):
        return crocoddyl.StateMultibody(self.get_agent_model())

    def get_agent_actuation_model(self):
        return crocoddyl.ActuationModelFloatingBase(self.get_state())

    def get_nu(self):
        return self.get_agent_actuation_model().nu
