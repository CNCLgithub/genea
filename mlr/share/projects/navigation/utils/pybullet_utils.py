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


class PyBulletForceRecord:
    def __init__(self, platform_id, nav_force: NavForce, sim_onset_time, sim_stint_time):
        """
        :param platform_id       : the ID of the platform onto which some force is to be applied
        :param nav_force         : the @NavForce object defining the force to be applied
        :param sim_onset_time    : the force onset time in seconds, measured from t=0
        :param sim_stint_time    : the duration (in seconds) for which the force must be applied
        """
        self._platform_id = platform_id
        self._nav_force = nav_force
        self._sim_onset_time = sim_onset_time
        self._sim_stint_time = sim_stint_time

    def get_platform_id(self):
        return self._platform_id

    def get_nav_force(self):
        return self._nav_force

    def get_sim_onset_time(self):
        return self._sim_onset_time

    def get_sim_stint_time(self):
        return self._sim_stint_time

    def get_sim_end_time(self):
        return self.get_sim_onset_time() + self.get_sim_stint_time()


class PyBulletUtils:
    def __init__(self, with_gui=False):
        self._platform_ids_list = []
        self._force_registry_list = []

        self._video_filepath = None

        if with_gui:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=ConfigUtils.PYBULLET_SIM_TIME_STEP, numSubSteps=1)
        pybullet.setTimeStep(ConfigUtils.PYBULLET_SIM_TIME_STEP)
        
    @staticmethod
    def _apply_forces(force_record: PyBulletForceRecord):
        platform_id = force_record.get_platform_id()
        nav_force = force_record.get_nav_force()

        force_mag = ConfigUtils.PYBULLET_FORCE_MAG_MULTIPLIER
        force_pos = nav_force.get_force_pose().get_position().get_position_as_np_array()
        force_rot = nav_force.get_force_pose().get_rotation().get_rotation_as_np_array() * force_mag

        if ConfigUtils.IS_DEBUG:
            pybullet.addUserDebugText("X", force_pos, textColorRGB=[1, 0, 0], textSize=1.5)

        pybullet.applyExternalForce(platform_id, -1, force_rot, force_pos, pybullet.LINK_FRAME)

    @staticmethod
    def close():
        pybullet.disconnect()

    def add_load_urdf(self, urdf_dirpath, urdf_rel_filepath, is_fixed=False):
        pybullet.setAdditionalSearchPath(urdf_dirpath)
        platform_id = pybullet.loadURDF(urdf_rel_filepath, useFixedBase=is_fixed)

        if not is_fixed:
            self._platform_ids_list.append(platform_id)

    def add_force_on_platform(self, platform_id, force: NavForce, sim_onset_time, sim_stint_time):
        self._force_registry_list.append(PyBulletForceRecord(platform_id, force, sim_onset_time, sim_stint_time))

    def run_simulation(self):
        sim_current_time = 0.0

        for tick in range(int(ConfigUtils.PYBULLET_SIM_TIME_TOTAL / ConfigUtils.PYBULLET_SIM_TIME_STEP)):
            if not pybullet.isConnected():
                continue

            for force_record in self._force_registry_list:
                if force_record.get_sim_onset_time() <= sim_current_time < force_record.get_sim_end_time():
                    self._apply_forces(force_record)

            pybullet.stepSimulation()

            sim_current_time += ConfigUtils.PYBULLET_SIM_TIME_STEP

            if ConfigUtils.NAV_MODEL_VIEW_DYNAMICS:
                time.sleep(ConfigUtils.PYBULLET_SIM_TIME_STEP)

    def get_platform_ids_list(self):
        return self._platform_ids_list

    @staticmethod
    def get_platform_pose(platform_id) -> NavPose:
        position, orientation = pybullet.getBasePositionAndOrientation(platform_id)
        return NavPose(NavPosition(*position), NavRotation(*pybullet.getEulerFromQuaternion(orientation)))
