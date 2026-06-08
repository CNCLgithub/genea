import numpy as np

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import StimuliConfig, PlatformConfig
from mlr.share.projects.navigation.utils.core_utils import NavPose, NavPosition, NavRotation
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.mjcf_utils import MJCFGenerator
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.mujoco_utils import MujocoUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.platform_utils import Platform, PlatformType


class StimuliCounter:
    def __init__(self, start: int = 0):
        self._value = start

    def __call__(self):
        return self._value

    def __iadd__(self, increment: int):
        self._value += increment
        return self


class StimuliSet:
    def __init__(self, stimuli_set_name):
        self._stimuli_set_name = stimuli_set_name
        self._stimuli_dirpath = PathUtils.get_stimuli_set_dirpath(stimuli_set_name)
        self._prepare_directories()

        self._stimuli_counter = StimuliCounter(0)

    def _prepare_directories(self):
        FileUtils.create_dir(self.get_stimuli_set_dirpath())

    def add_stimulus(self):
        self._stimuli_counter += 1

    def get_stimuli_set_name(self):
        return self._stimuli_set_name

    def get_stimuli_count(self) -> int:
        return self._stimuli_counter()

    def get_stimuli_set_dirpath(self):
        return self._stimuli_dirpath

    @staticmethod
    def get_available_platform_types_list():
        platform_type = FileUtils.get_files_in_directory(PathUtils.get_platforms_dirpath())
        return [FileUtils.get_basename(name, False)[0] for name in platform_type]


class Stimulus:
    def __init__(self, stimuli_set: StimuliSet):
        self._stim_set = stimuli_set

        self._stim_name = self._stim_set.get_stimuli_set_name() + "_" + str(self._stim_set.get_stimuli_count())
        self._stim_set.add_stimulus()

        self._stim_platforms_dict = {}
        self._stim_platforms_counter = 0
        self._stim_mjcf_generator = MJCFGenerator(self.get_stimulus_dirpath())

        self._prepare_stimulus_directories()

    def _prepare_stimulus_directories(self):
        FileUtils.create_dir(self.get_stimulus_dirpath())
        FileUtils.create_dir(self.get_stimulus_meshes_dirpath())

    def add_platform(self, platform_type, platform_pose: NavPose):
        platform = Platform(str(self._stim_platforms_counter), platform_type, platform_pose)

        self._stim_platforms_counter += 1
        self._stim_platforms_dict[platform.get_platform_id()] = platform

        # add mesh
        mesh_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), platform.get_platform_mesh_filename())
        FileUtils.copy_file(mesh_filepath, self.get_stimulus_meshes_dirpath())

        # add mjcf
        self._stim_mjcf_generator.add_body(platform.get_platform_id(),
                                           platform.get_platform_mesh_name(),
                                           platform.get_platform_mesh_filename(),
                                           platform_pose.get_position().get_position_as_str(),
                                           platform_pose.get_rotation().get_rotation_as_str(),
                                           platform.get_mass())

    def add_ground(self, ground_pose=NavPose(NavPosition(0.0, 0.0, -PlatformConfig.PLATFORM_HEIGHT))):
        ground_size = 10
        ground_height = 0.25
        ground_pose.get_position().add_z(-ground_height)
        ground_pose_as_str = ground_pose.get_position().get_position_as_str()
        self._stim_mjcf_generator.add_ground(ground_pose_as_str, f"{ground_size} {ground_size} {ground_height}")

    def add_camera(self):
        camera_pos_str = f"{self.get_center_x()} -30 10"
        self._stim_mjcf_generator.add_camera(camera_pos_str, "1.000 -0.016 -0.000 0.005 0.328 0.945")

    def load_stimuli_from_library(self, mjcf_filepath=None):
        if mjcf_filepath is None:
            mjcf_filepath = self._stim_mjcf_generator.get_mjcf_filepath()

        mj = MujocoUtils(mjcf_filepath)
        platform_ids_list = mj.get_body_names_list()
        platform_types_list = mj.get_body_types_list()
        for platform_id, platform_type in zip(platform_ids_list, platform_types_list):
            platform_pos, platform_rot = mj.get_body_pose_by_name(platform_id)
            platform_pose = NavPose(NavPosition(*platform_pos), NavRotation(*platform_rot))
            self._stim_platforms_dict[platform_id] = Platform(platform_id, platform_type, platform_pose)

    def save_to_mjcf(self):
        self._stim_mjcf_generator.save_mjcf()

    def visualize(self):
        mj = MujocoUtils(self._stim_mjcf_generator.get_mjcf_filepath())
        mj.visualize()

    @staticmethod
    def get_root_platform_pose() -> NavPose:
        return NavPose(NavPosition(0.0, 0.0, -PlatformConfig.PLATFORM_HEIGHT))

    def get_next_platform_pose(self, query_platform_type, ref_platform_id, delta_x, delta_y) -> NavPose:
        if ref_platform_id not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {ref_platform_id} not found.")
            assert False

        next_x = self.get_platform(ref_platform_id).get_platform_pose().get_position().get_x()
        next_y = self.get_platform(ref_platform_id).get_platform_pose().get_position().get_y()

        ref_surface_xy = self.get_platform_top_surface_xy(ref_platform_id)
        next_x += ref_surface_xy[0] / 2

        next_x += StimuliConfig.MIN_GAP_X + delta_x
        next_y += delta_y

        query_surface_xy = Platform.get_platform_top_surface_xy(query_platform_type)
        next_x += query_surface_xy[0] / 2

        return NavPose(NavPosition(next_x, next_y, -PlatformConfig.PLATFORM_HEIGHT))

    def get_goal_platform_pose(self, delta_x, delta_y) -> NavPose:
        query_surface_xy = Platform.get_platform_top_surface_xy(PlatformType.get_goal())

        goal_x = self.get_max_x()
        goal_x += StimuliConfig.MIN_GAP_X
        goal_x += query_surface_xy[0] / 2
        goal_x += delta_x

        goal_y = 0.0
        goal_y += delta_y

        return NavPose(NavPosition(goal_x, goal_y, -PlatformConfig.PLATFORM_HEIGHT))

    def get_stimulus_name(self):
        return self._stim_name

    def get_stimulus_dirpath(self):
        return PathUtils.join(self._stim_set.get_stimuli_set_dirpath(), self._stim_name)

    def get_stimulus_meshes_dirpath(self):
        return PathUtils.join(self.get_stimulus_dirpath(), "meshes")

    def get_platform_ids_list(self):
        return list(self._stim_platforms_dict.keys())

    def get_platform_id_by_index(self, query_index):
        if query_index == -1:
            return self.get_platform_ids_list()[-1]

        if query_index < len(self.get_platform_ids_list()):
            return self.get_platform_ids_list()[query_index]

        Msg.print_error(f"ERROR [Stimuli]: platform index {query_index} not found.")
        assert False

    def get_platform(self, platform_id) -> Platform:
        if platform_id not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_id} not found.")
            assert False

        return self._stim_platforms_dict[platform_id]

    def get_platform_top_surface_xy(self, platform_id):
        if platform_id not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {platform_id} not found.")
            assert False

        return Platform.get_platform_top_surface_xy(self.get_platform(platform_id).get_platform_type())

    def get_gap_between_platform_centers(self, platform_id1, platform_id2):
        if platform_id1 not in self._stim_platforms_dict or platform_id2 not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_id1} or {platform_id2} not found.")
            assert False

        pos_p1 = self._stim_platforms_dict[platform_id1].get_platform_pose().get_position().get_position_as_list()
        pos_p2 = self._stim_platforms_dict[platform_id2].get_platform_pose().get_position().get_position_as_list()
        return ComputeUtils.compute_l2_distance(pos_p1, pos_p2)

    def get_gap_between_platform_edges(self, platform_id1, platform_id2):
        if platform_id1 not in self._stim_platforms_dict or platform_id2 not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_id1} or {platform_id2} not found.")
            assert False

        gap_between_centers = self.get_gap_between_platform_centers(platform_id1, platform_id2)
        if gap_between_centers == 0.0:
            return 0.0

        pos_p1 = self._stim_platforms_dict[platform_id1].get_platform_pose().get_position().get_position_as_np_array()
        pos_p2 = self._stim_platforms_dict[platform_id2].get_platform_pose().get_position().get_position_as_np_array()
        unit_vec = (pos_p2 - pos_p1) / np.linalg.norm(pos_p2 - pos_p1)

        surface_xy_p1 = self.get_platform_top_surface_xy(platform_id1)
        surface_xy_p2 = self.get_platform_top_surface_xy(platform_id2)

        half_surface_p1 = np.abs(unit_vec[:2]) @ (np.array(surface_xy_p1) / 2)
        half_surface_p2 = np.abs(unit_vec[:2]) @ (np.array(surface_xy_p2) / 2)

        return gap_between_centers - half_surface_p1 - half_surface_p2

    def get_center_x(self):
        min_platform_list = []
        max_platform_list = []
        for platform_id in self.get_platform_ids_list():
            platform_x = self.get_platform(platform_id).get_platform_pose().get_position().get_x()
            surface_xy = self.get_platform_top_surface_xy(platform_id)
            min_platform_list.append(platform_x - surface_xy[0] / 2)
            max_platform_list.append(platform_x + surface_xy[0] / 2)

        min_x = min(min_platform_list)
        max_x = max(max_platform_list)

        return min_x + (max_x - min_x) / 2.0

    def get_max_x(self):
        max_platform_list = []
        for platform_id in self.get_platform_ids_list():
            platform_x = self.get_platform(platform_id).get_platform_pose().get_position().get_x()
            surface_xy = self.get_platform_top_surface_xy(platform_id)
            max_platform_list.append(platform_x + surface_xy[0] / 2)
        return max(max_platform_list)
