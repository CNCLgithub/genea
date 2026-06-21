import mujoco as mj
import mujoco.viewer as mjv
import numpy as np

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import MujocoConfig, NavConfig
from mlr.share.projects.navigation.utils.core_utils import NavForce


class MujocoLandmark:
    def __init__(self, landmark_pos, landmark_vec):
        self._landmark_pos = landmark_pos
        self._landmark_vec = landmark_vec

    def add_force_arrow(self, scene):
        geom = scene.geoms[scene.ngeom]
        mj.mjv_initGeom(geom,
                        type=mj.mjtGeom.mjGEOM_ARROW1,
                        size=[0.1, 0.1, 0.5],
                        pos=self.get_landmark_pos(),
                        mat=self.get_landmark_mat(),
                        rgba=np.array([1, 0, 0, 1]))

        scene.ngeom += 1

    def get_landmark_pos(self):
        return self._landmark_pos

    def get_landmark_vec(self):
        return self._landmark_vec

    def get_landmark_mat(self):
        normalized = self._landmark_vec / (np.linalg.norm(self._landmark_vec) + 1e-12)
        up = np.array([0, 0, 1])

        if abs(np.dot(up, normalized)) > 0.9:
            up = np.array([0, 1, 0])

        x = np.cross(up, normalized)
        x /= np.linalg.norm(x) + 1e-8
        y = np.cross(normalized, x)

        return np.column_stack([x, y, normalized]).flatten()


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
        self._model = mj.MjModel.from_xml_path(stim_mjcf_filepath)  # noqa
        self._data = mj.MjData(self._model)

        self._landmark_id_list = []
        self._platform_ids_list = []
        self._external_forces_list_by_time = []

        self._video_filepath = None

    def visualize(self):
        with mjv.launch_passive(self._model, self._data) as viewer:
            viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = self._model.camera("camera").id

            for landmark in self._landmark_id_list:
                landmark.add_force_arrow(viewer.user_scn)

            while viewer.is_running():
                mj.mj_step(self._model, self._data)  # noqa
                viewer.sync()

    def get_body_names_list(self):
        body_names_list = []
        for body_index in range(1, self._model.nbody):
            body_name = mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_BODY, body_index)
            if body_name:
                body_names_list.append(body_name)
        return body_names_list

    def get_body_meshes_list(self):
        body_types_list = []
        for body_index in range(1, self._model.nbody):
            body_geom = self._model.body_geomadr[body_index]
            mesh_id = self._model.geom_dataid[body_geom]
            body_type = mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_MESH, mesh_id)
            if body_type:
                body_types_list.append(body_type)
        return body_types_list

    def get_body_pose_by_name(self, body_name: str):
        return_pos_as_list = []
        return_rot_as_list = []
        for body_index in range(1, self._model.nbody):
            if mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_BODY, body_index) == body_name:
                pos = self._model.body_pos[body_index]
                rot = self._model.body_quat[body_index]
                rot = ComputeUtils.convert_quat_to_euler([rot[1], rot[2], rot[3], rot[0]])
                return_pos_as_list = pos
                return_rot_as_list = rot
                break
        return return_pos_as_list, return_rot_as_list

    def register_external_forces(self, nav_forces_list):
        self._external_forces_list_by_time.append(nav_forces_list)

    def simulate(self):
        for forces_list in self._external_forces_list_by_time:
            self._data.qfrc_applied[:] = 0.0
            self._data.xfrc_applied[:] = 0.0
            for nav_force in forces_list:
                force_mag = MujocoConfig.FORCE_SCALE
                force_pos = nav_force.get_force_pose().get_position().get_position_as_np_array()
                force_vec = nav_force.get_force_pose().get_rotation().get_rotation_as_np_array() * force_mag

                if NavConfig.DEBUG_DYNAMICS:
                    self._landmark_id_list.append(MujocoLandmark(force_pos, force_vec))
                    continue

                mj.mj_applyFT(self._model, self._data, force_vec, np.zeros(3), force_pos)

    def close(self):
        mj.mj_resetData(self._model, self._data)
