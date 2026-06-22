import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
import time

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import MujocoConfig, NavConfig
from mlr.share.projects.navigation.utils.core_utils import NavForce
from mlr.share.projects.navigation.utils.navigation_utils import NavTaskRegistry


class MujocoLandmark:
    def __init__(self, landmark_pos, landmark_vec):
        self._landmark_pos = landmark_pos
        self._landmark_vec = landmark_vec

    def add_force_arrow(self, scene):
        geom = scene.geoms[scene.ngeom]

        start_pos = self.get_landmark_pos()
        final_pos = self.get_landmark_pos() + self.get_landmark_vec()
        mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_ARROW, 0.05, start_pos, final_pos)
        geom.rgba[:] = [1, 0, 0, 1]
        scene.ngeom += 1

        # geom = scene.geoms[scene.ngeom]
        # mj.mjv_initGeom(geom,
        #                 type=mj.mjtGeom.mjGEOM_SPHERE,
        #                 size=[0.2, 0.2, 0.2],
        #                 pos=self.get_landmark_pos(),
        #                 mat=np.eye(3).flatten(),
        #                 rgba=np.array([1, 0, 0, 1]))
        # scene.ngeom += 1
        #
        # geom = scene.geoms[scene.ngeom]
        # mj.mjv_initGeom(geom,
        #                 type=mj.mjtGeom.mjGEOM_SPHERE,
        #                 size=[0.1, 0.1, 0.1],
        #                 pos=final_pos,
        #                 mat=np.eye(3).flatten(),
        #                 rgba=np.array([1, 1, 0, 1]))
        # scene.ngeom += 1

    def get_landmark_pos(self):
        return self._landmark_pos

    def get_landmark_vec(self):
        return self._landmark_vec


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

        self._viewer = None

        self._landmark_id_list = []
        self._platform_ids_list = []

        self._video_filepath = None

    def hide_all(self):
        for i in range(self._model.ngeom):
            self._model.geom_rgba[i, 3] = 0.0

    def _register_task(self, force_pos, force_vec, platform_name):
        if NavConfig.DEBUG_DYNAMICS:
            self._landmark_id_list.append(MujocoLandmark(force_pos, force_vec))
            return

        mj_body_id = mj.mj_name2id(self._model, mj.mjtObj.mjOBJ_BODY, platform_name)
        self._model.body_mass[mj_body_id] = 1.
        mj.mj_applyFT(self._model, self._data, force_vec, np.zeros(3), force_pos, mj_body_id, self._data.qfrc_applied)

    def _step(self, task_registry: NavTaskRegistry):
        if task_registry.get_force_left() is not None:
            left_pos = task_registry.get_force_left_pos()
            left_vec = task_registry.get_force_left_vec()
            left_loc = task_registry.get_platform_name_left()
            self._register_task(left_pos, left_vec, left_loc)

        if task_registry.get_force_right() is not None:
            right_pos = task_registry.get_force_right_pos()
            right_vec = task_registry.get_force_right_vec()
            right_loc = task_registry.get_platform_name_right()
            self._register_task(right_pos, right_vec, right_loc)

    def _close_viewer(self):
        if self._viewer is None:
            return

        viewer = self._viewer
        self._viewer = None

        if hasattr(viewer, "is_running") and viewer.is_running():
            viewer.close()

    def simulate(self, nav_task_registry_list: list[NavTaskRegistry]):
        if NavConfig.VIEW_DYNAMICS:
            self.init_viewer()

        iter_num = 0
        start_time = time.time()
        while True:
            if self._data.time >= MujocoConfig.SIMULATION_TIME:
                self._close_viewer()
                break

            if iter_num < len(nav_task_registry_list):
                self._data.qfrc_applied[:] = 0.0
                self._data.xfrc_applied[:] = 0.0
                self._step(nav_task_registry_list[iter_num])
                iter_num += 1

            if self._viewer is not None and self._viewer.is_running():
                if time.time() - start_time >= 5.0:
                    self._close_viewer()
                    break
                self._viewer.sync()
                time.sleep(0.01)

            mj.mj_step(self._model, self._data)  # noqa

    def reset(self):
        mj.mj_resetData(self._model, self._data)

    def visualize(self):
        self.init_viewer()

        for landmark in self._landmark_id_list:
            landmark.add_force_arrow(self._viewer.user_scn)

        while self._viewer.is_running():
            self._viewer.sync()

    def init_viewer(self):
        if self._viewer is None:
            self._viewer = mjv.launch_passive(self._model, self._data)
            self._viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED
            self._viewer.cam.fixedcamid = self._model.camera("camera").id
        return self._viewer

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
