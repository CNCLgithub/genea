import pybullet
import time

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavForce, NavPose, NavPosition, NavRotation


class PyBulletRobot:
    def __init__(self, pybullet_id, pin_model):
        self.nq = pin_model.nq
        self.nv = pin_model.nv
        self.robot_id = pybullet_id
        self.pin_model = pin_model

        bullet_joint_map = {}
        for joint in range(pybullet.getNumJoints(self.robot_id)):
            bullet_joint_map[pybullet.getJointInfo(self.robot_id, joint)[1].decode("UTF-8")] = joint


class PyBulletUtils:
    def __init__(self, with_gui=False):
        self._platform_ids_list = []

        self._video_filepath = None

        if with_gui:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=ConfigUtils.PYBULLET_SIMULATION_TIME_STEP, numSubSteps=1)
        pybullet.setTimeStep(ConfigUtils.PYBULLET_SIMULATION_TIME_STEP)

    def add_load_urdf(self, urdf_dirpath, urdf_rel_filepath, is_fixed=False):
        pybullet.setAdditionalSearchPath(urdf_dirpath)
        platform_id = pybullet.loadURDF(urdf_rel_filepath, useFixedBase=is_fixed)

        if not is_fixed:
            self._platform_ids_list.append(platform_id)

    def get_platform_ids_list(self):
        return self._platform_ids_list

    @staticmethod
    def get_pose(platform_id) -> NavPose:
        position, orientation = pybullet.getBasePositionAndOrientation(platform_id)
        return NavPose(NavPosition(*position), NavRotation(*pybullet.getEulerFromQuaternion(orientation)))

    @staticmethod
    def run_simulation(sim_time_secs):
        start_time = time.time()
        while pybullet.isConnected() and (time.time() - start_time < sim_time_secs):
            pybullet.stepSimulation()
            time.sleep(1. / 240.)

    @staticmethod
    def close():
        pybullet.disconnect()

    @staticmethod
    def add_force_to_object(platform_id, force: NavForce):
        force_mag = ComputeUtils.sample_uniform(1.0, ConfigUtils.PYBULLET_FORCE_MAG_MULTIPLIER, 1)
        force_pos = force.get_force_pose().get_position().get_position_as_np_array()
        force_rot = force.get_force_pose().get_rotation().get_rotation_as_np_array() * force_mag

        pybullet.addUserDebugText("X", force_pos, textColorRGB=[1, 0, 0], textSize=1.5)

        pybullet.applyExternalForce(platform_id, -1, force_rot, force_pos, pybullet.LINK_FRAME)

    @staticmethod
    def get_pose_diff(start_pose, final_pose):
        start_pos = start_pose.get_position().get_position_as_np_array()
        final_pos = final_pose.get_position().get_position_as_np_array()
        pos_diff = ComputeUtils.compute_l2_distance(start_pos, final_pos)

        start_rot = start_pose.get_rotation().get_rotation_as_np_array()
        final_rot = final_pose.get_rotation().get_rotation_as_np_array()
        rot_diff = ComputeUtils.compute_angle_magnitude(start_rot, final_rot)

        return pos_diff, rot_diff
