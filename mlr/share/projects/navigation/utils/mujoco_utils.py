from __future__ import annotations

import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
import time

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import MujocoConfig, NavConfig
from mlr.share.projects.navigation.utils.msg_utils import Msg
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

    def add_agent_steps(self, scene):
        geom = scene.geoms[scene.ngeom]
        mj.mjv_initGeom(geom,
                        type=mj.mjtGeom.mjGEOM_SPHERE,
                        size=[0.2, 0.2, 0.2],
                        pos=self.get_landmark_pos(),
                        mat=np.eye(3).flatten(),
                        rgba=np.array([1, 0, 0, 1]))
        scene.ngeom += 1

    def get_landmark_pos(self):
        return self._landmark_pos

    def get_landmark_vec(self):
        return self._landmark_vec


class MujocoUtils:
    def __init__(self, stim_mjcf_filepath):
        self._model = mj.MjModel.from_xml_path(stim_mjcf_filepath)  # noqa
        self._data = mj.MjData(self._model)

        self._viewer = None
        self._video_filepath = None

        self._landmark_id_list = []

    def _register_landmark(self, force_pos, force_vec):
        self._landmark_id_list.append(MujocoLandmark(force_pos, force_vec))

    def _register_force(self, force_pos, force_vec, platform_name):
        mj_body_id = mj.mj_name2id(self._model, mj.mjtObj.mjOBJ_BODY, platform_name)
        mj.mj_applyFT(self._model, self._data, force_vec, np.zeros(3), force_pos, mj_body_id, self._data.qfrc_applied)

    def _unpack_task_registry(self, task_registry: NavTaskRegistry):
        if task_registry.has_left_force():
            left_pos = task_registry.get_force_left_pos()
            left_vec = task_registry.get_force_left_vec()
            left_loc = task_registry.get_platform_name_left()
            if NavConfig.DEBUG_DYNAMICS:
                self._register_landmark(left_pos, left_vec)
            else:
                self._register_force(left_pos, left_vec, left_loc)

        if task_registry.has_right_force():
            right_pos = task_registry.get_force_right_pos()
            right_vec = task_registry.get_force_right_vec()
            right_loc = task_registry.get_platform_name_right()
            if NavConfig.DEBUG_DYNAMICS:
                self._register_landmark(right_pos, right_vec)
            else:
                self._register_force(right_pos, right_vec, right_loc)

    def _init_viewer(self):
        if self._viewer is None:
            self._viewer = mjv.launch_passive(self._model, self._data)
            self._viewer.cam.type = mj.mjtCamera.mjCAMERA_FIXED
            self._viewer.cam.fixedcamid = self._model.camera("camera").id
        return self._viewer

    def _close_viewer(self):
        if self._viewer is None:
            return

        viewer = self._viewer
        self._viewer = None

        if hasattr(viewer, "is_running") and viewer.is_running():
            viewer.close()
            
    def _add_transparency(self, alpha=0.0):
        for i in range(self._model.ngeom):
            self._model.geom_rgba[i, 3] = alpha

    def visualize(self, nav_task_registry_list=None):
        self._init_viewer()
        self._add_transparency(0.4)

        if nav_task_registry_list is not None:
            for nav_task_registry in nav_task_registry_list:
                self._unpack_task_registry(nav_task_registry)

            for landmark in self._landmark_id_list:
                landmark.add_agent_steps(self._viewer.user_scn)

        while self._viewer.is_running():
            self._viewer.sync()

    def simulate(self, nav_task_registry_list: list[NavTaskRegistry]):
        start_body_pos_list = [self.get_init_pose(body_name) for body_name in self.get_body_names_list()]

        if NavConfig.VIEW_DYNAMICS:
            self._init_viewer()

        iter_num = 0
        start_time = time.time()
        while True:
            if self._data.time >= MujocoConfig.SIMULATION_TIME:
                self._close_viewer()
                break

            if iter_num < len(nav_task_registry_list):
                self._data.qfrc_applied[:] = 0.0
                self._data.xfrc_applied[:] = 0.0
                self._unpack_task_registry(nav_task_registry_list[iter_num])
                iter_num += 1

            if self._viewer is not None and self._viewer.is_running():
                if time.time() - start_time >= 5.0:
                    self._close_viewer()
                    break
                self._viewer.sync()
                time.sleep(0.01)

            mj.mj_step(self._model, self._data)  # noqa

        final_body_pos_list = [self.get_curr_pose(body_name) for body_name in self.get_body_names_list()]

        for start, final in zip(start_body_pos_list, final_body_pos_list):
            pos_diff = np.linalg.norm(start[0] - final[0]).item()
            rot_diff = np.linalg.norm(start[1] - final[1]).item()
            if pos_diff > NavConfig.DYNAMICS_TOLERANCE_POS or rot_diff > NavConfig.DYNAMICS_TOLERANCE_ROT:
                Msg.print_info(f"ERROR [MujocoUtils]: pos -- {pos_diff} | rot -- {rot_diff}")
                return Msg.FAILURE
        return Msg.SUCCESS

    def reset(self):
        mj.mj_resetData(self._model, self._data)

    def get_body_names_list(self):
        body_names_list = []
        for body_index in range(1, self._model.nbody):
            body_name = mj.mj_id2name(self._model, mj.mjtObj.mjOBJ_BODY, body_index)
            if body_name:
                body_names_list.append(body_name)
        return body_names_list

    def get_init_pose(self, body_name: str):
        body_id = mj.mj_name2id(self._model, mj.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            Msg.print_error(f"ERROR [MujocoUtils]: body {body_name} not found")
            assert False

        pos = self._model.body_pos[body_id].copy()
        rot = self._model.body_quat[body_id].copy()
        rot = ComputeUtils.convert_quat_to_euler([rot[1], rot[2], rot[3], rot[0]])

        return pos, rot

    def get_curr_pose(self, body_name: str):
        body_id = mj.mj_name2id(self._model, mj.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            Msg.print_error(f"ERROR [MujocoUtils]: body {body_name} not found")
            assert False

        pos = self._data.xpos[body_id].copy()
        rot = self._data.xquat[body_id].copy()
        rot = ComputeUtils.convert_quat_to_euler([rot[1], rot[2], rot[3], rot[0]])

        return pos, rot
