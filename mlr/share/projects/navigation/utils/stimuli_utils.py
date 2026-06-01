import numpy as np

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import StimuliConfig
from mlr.share.projects.navigation.utils.core_utils import NavPose, NavPosition, NavRotation, NavColor
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.mjcf_utils import MJCFGenerator
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.mujoco_utils import MujocoUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.platform_utils import Platform


class StimuliSet:
    def __init__(self, stimuli_set_name):
        self._stimuli_set_name = stimuli_set_name
        self._stimuli_dirpath = PathUtils.get_stimuli_set_dirpath(stimuli_set_name)
        self._prepare_directories()

    def _prepare_directories(self):
        FileUtils.create_dir(self.get_stimuli_set_dirpath())

    def get_stimuli_set_name(self):
        return self._stimuli_set_name

    def get_stimuli_set_dirpath(self):
        return self._stimuli_dirpath

    @staticmethod
    def get_available_platform_types_list():
        platform_type = FileUtils.get_files_in_directory(PathUtils.get_platforms_dirpath())
        return [FileUtils.get_basename(name, False)[0] for name in platform_type]


class Stimulus:
    GROUND_COLOR = NavColor("dark_gray", (.9, .9, .9, 1))
    PLATFORM_COLOR = NavColor("light_gray", (.7, .7, .7, 1))

    def __init__(self, stimulus_name, stimuli_set: StimuliSet):
        self._stimulus_name = stimulus_name
        self._stimuli_set = stimuli_set

        self._stim_platforms_dict = {}
        self._stim_mjcf_generator = MJCFGenerator(self.get_stimulus_dirpath())

        self._prepare_stimulus_directories()

    def _prepare_stimulus_directories(self):
        FileUtils.create_dir(self.get_stimulus_dirpath())
        FileUtils.create_dir(self.get_stimulus_meshes_dirpath())

    def _add_platform(self, platform_type, platform_pose: NavPose, platform_color: NavColor):
        platform_name = self._get_platform_name(platform_type, len(self._stim_platforms_dict.keys()) + 1)

        platform = Platform(platform_name, platform_type, platform_pose, platform_color)
        self._stim_platforms_dict[platform_name] = platform

        # add mesh
        platform_mesh_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), platform.get_platform_filename())
        FileUtils.copy_file(platform_mesh_filepath, self.get_stimulus_meshes_dirpath())

        # add mjcf
        self._stim_mjcf_generator.add_body(platform.get_platform_name(),
                                           platform.get_platform_type(),
                                           platform.get_platform_filename(),
                                           platform_pose.get_position().get_position_as_str(),
                                           platform_pose.get_rotation().get_rotation_as_str(),
                                           platform_color.get_color_as_str(),
                                           platform.get_mass())

    def add_ground_plane(self, ground_pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        ground_size = 10
        ground_height = 0.25

        ground_pose.get_position().add_z(-ground_height)
        ground_pose_as_str = ground_pose.get_position().get_position_as_str()

        size_str = f"{ground_size} {ground_size} {ground_height}"

        self._stim_mjcf_generator.add_ground(ground_pose_as_str, size_str, Stimulus.GROUND_COLOR.get_color_as_str())

    def add_root_platform(self, pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        self._add_platform(Platform.PLATFORM_ROOT, pose, Stimulus.PLATFORM_COLOR)

    def add_goal_platform(self, pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        self._add_platform(Platform.PLATFORM_GOAL, pose, Stimulus.PLATFORM_COLOR)

    def add_platform(self, platform_filename, platform_pose):
        self._add_platform(platform_filename, platform_pose, Stimulus.PLATFORM_COLOR)

    def add_camera(self):
        camera_pos_str = f"{self.get_center_x()} -30 10"
        self._stim_mjcf_generator.add_camera(camera_pos_str, "1.000 -0.016 -0.000 0.005 0.328 0.945")

    def load_stimuli_from_library(self, mjcf_filepath=None):
        if mjcf_filepath is None:
            mjcf_filepath = self._stim_mjcf_generator.get_mjcf_filepath()

        mj = MujocoUtils(mjcf_filepath)
        platform_names_list = mj.get_body_names_list()
        platform_types_list = mj.get_body_types_list()
        for platform_name, platform_type in zip(platform_names_list, platform_types_list):
            platform_pos, platform_rot = mj.get_body_pose_by_name(platform_name)
            platform_pose = NavPose(NavPosition(*platform_pos), NavRotation(*platform_rot))
            self._stim_platforms_dict[platform_name] = Platform(platform_name, platform_type, platform_pose)

    def save_to_mjcf(self):
        self._stim_mjcf_generator.save_mjcf()

    def visualize(self):
        mj = MujocoUtils(self._stim_mjcf_generator.get_mjcf_filepath())
        mj.visualize()

    @staticmethod
    def _get_platform_name(platform_type, platform_index):
        return platform_type + "_" + str(platform_index)

    def get_stimulus_name(self):
        return self._stimulus_name

    def get_stimulus_dirpath(self):
        return PathUtils.join(self._stimuli_set.get_stimuli_set_dirpath(), self._stimulus_name)

    def get_stimulus_meshes_dirpath(self):
        return PathUtils.join(self.get_stimulus_dirpath(), "meshes")

    def get_platform_keys_list(self):
        return list(sorted(list(self._stim_platforms_dict.keys()), key=lambda x: int(x.split("_")[-1])))

    def get_platform_key_by_index(self, query_index):
        if query_index == -1:
            return self.get_platform_keys_list()[-1]

        if query_index < len(self.get_platform_keys_list()):
            return self.get_platform_keys_list()[query_index]

        Msg.print_error(f"ERROR [Stimuli]: platform index {query_index} not found.")
        assert False

    def get_platform_by_key(self, platform_key):
        if platform_key not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_key} not found.")
            assert False

        return self._stim_platforms_dict[platform_key]

    def get_platform_top_surface_xy(self, platform_key):
        if platform_key not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {platform_key} not found.")
            assert False

        return Platform.get_platform_top_surface_xy(self._stim_platforms_dict[platform_key].get_platform_type())

    def get_gap_between_platforms(self, platform_key1, platform_key2):
        if platform_key1 not in self._stim_platforms_dict or platform_key2 not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_key1} or {platform_key2} not found.")
            assert False

        p1 = self._stim_platforms_dict[platform_key1].get_platform_pose().get_position().get_position_as_list()
        p2 = self._stim_platforms_dict[platform_key2].get_platform_pose().get_position().get_position_as_list()

        return ComputeUtils.compute_l2_distance(p1, p2)

    def get_gap_to_next_platform(self, platform_key):
        if platform_key not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {platform_key} not found.")
            assert False

        platform_x, _ = self.get_platform_top_surface_xy(platform_key)

        return platform_x / 2.0 + StimuliConfig.PLATFORM_GAP

    def get_center_x(self):
        min_x = np.inf
        max_x = -np.inf

        platform_key_list = self.get_platform_keys_list()
        for platform_key in platform_key_list:
            platform = self.get_platform_by_key(platform_key)
            platform_x_pos = platform.get_platform_pose().get_position().get_x()
            if platform_x_pos < min_x:
                min_x = platform_x_pos
            if platform_x_pos > max_x:
                max_x = platform_x_pos

        return min_x + (max_x - min_x) / 2.0
