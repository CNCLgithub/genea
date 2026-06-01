import mujoco as mj
import mujoco.viewer as mjv
import time

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import CoreConfig
from mlr.share.projects.navigation.utils.core_utils import NavForce, NavPose, NavPosition, NavRotation


class MujocoForceRecord:
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


class MujocoUtils:
    def __init__(self, stim_mjcf_filepath):
        self._model = mj.MjModel.from_xml_path(stim_mjcf_filepath)
        self._data = mj.MjData(self._model)

        self._platform_ids_list = []
        self._force_registry_list = []

        self._video_filepath = None

    def visualize(self):
        with mjv.launch_passive(self._model, self._data) as viewer:
            viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = self._model.camera("camera").id
            while viewer.is_running():
                mj.mj_step(self._model, self._data)
                viewer.sync()

    def get_body_names_list(self):
        body_names_list = []
        for body_index in range(1, self._model.nbody):
            body_name = mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_BODY, body_index)
            if body_name:
                body_names_list.append(body_name)
        return body_names_list

    def get_body_types_list(self):
        body_types_list = []
        for body_index in range(1, self._model.nbody):
            body_geom = self._model.body_geomadr[body_index]
            mesh_id = self._model.geom_dataid[body_geom]
            body_type = mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_MESH, mesh_id)
            if body_type:
                body_types_list.append(body_type)
        return body_types_list

    def get_body_pose_by_name(self, body_name: str):
        return_pos_list = []
        return_rot_list = []
        for body_index in range(1, self._model.nbody):
            if mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_BODY, body_index) == body_name:
                pos = self._model.body_pos[body_index]
                rot = self._model.body_quat[body_index]
                rot = ComputeUtils.convert_quat_to_euler([rot[1], rot[2], rot[3], rot[0]])
                return_pos_list.append(pos)
                return_rot_list.append(rot)
        return return_pos_list, return_rot_list

    @staticmethod
    def _apply_forces(force_record: MujocoForceRecord):
        platform_id = force_record.get_platform_id()
        nav_force = force_record.get_nav_force()

        force_mag = CoreConfig.PYBULLET_FORCE_MAG_MULTIPLIER
        force_pos = nav_force.get_force_pose().get_position().get_position_as_np_array()
        force_rot = nav_force.get_force_pose().get_rotation().get_rotation_as_np_array() * force_mag

        body_xmat = self._data.xmat[platform_id].reshape(3, 3)
        world_force = body_xmat @ force_vec

        if CoreConfig.IS_DEBUG:
            pybullet.addUserDebugText("X", force_pos, textColorRGB=[1, 0, 0], textSize=1.5)

        mj.mj_applyFT(platform_id, -1, force_rot, force_pos, pybullet.LINK_FRAME)

    def close(self):
        mj.mj_resetData(self.model, self.data)

    def add_force_on_platform(self, platform_id, force: NavForce, sim_onset_time, sim_stint_time):
        self._force_registry_list.append(MujocoForceRecord(platform_id, force, sim_onset_time, sim_stint_time))

    def run_simulation(self):
        sim_current_time = 0.0

        for tick in range(int(CoreConfig.PYBULLET_SIM_TIME_TOTAL / CoreConfig.PYBULLET_SIM_TIME_STEP)):
            if not pybullet.isConnected():
                continue

            for force_record in self._force_registry_list:
                if force_record.get_sim_onset_time() <= sim_current_time < force_record.get_sim_end_time():
                    self._apply_forces(force_record)

            pybullet.stepSimulation()

            sim_current_time += CoreConfig.PYBULLET_SIM_TIME_STEP

            if CoreConfig.NAV_MODEL_VIEW_DYNAMICS:
                time.sleep(CoreConfig.PYBULLET_SIM_TIME_STEP)

    def get_platform_ids_list(self):
        return self._platform_ids_list

    @staticmethod
    def get_platform_pose(platform_id) -> NavPose:
        position, orientation = pybullet.getBasePositionAndOrientation(platform_id)
        return NavPose(NavPosition(*position), NavRotation(*pybullet.getEulerFromQuaternion(orientation)))
