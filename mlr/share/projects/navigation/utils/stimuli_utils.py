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

    def get_next_stimulus_name(self):
        return self.get_stimuli_set_name() + "_" + str(self.get_stimuli_count())

    @staticmethod
    def get_available_platform_types_list():
        platform_type = FileUtils.get_files_in_directory(PathUtils.get_platforms_dirpath())
        return [FileUtils.get_basename(name, False)[0] for name in platform_type]


class Stimulus:
    def __init__(self, stimuli_set: StimuliSet, stimulus_name=None):
        self._stim_set = stimuli_set

        self._stim_name = self._stim_set.get_next_stimulus_name()
        if stimulus_name:
            self._stim_name = stimulus_name

        self._stim_set.add_stimulus()

        self._stim_platforms_dict = {}
        self._stim_platforms_counter = 0
        self._stim_mjcf_generator = MJCFGenerator(self.get_stimulus_mjcf_filepath())

        self._prepare_stimulus_directories()

    def _prepare_stimulus_directories(self):
        FileUtils.create_dir(self.get_stimulus_dirpath())
        FileUtils.create_dir(self.get_stimulus_meshes_dirpath())

    def add_platform(self, platform_type, platform_pose: NavPose):
        platform = Platform(str(self._stim_platforms_counter), platform_type, platform_pose)

        self._stim_platforms_counter += 1
        self._stim_platforms_dict[platform.get_platform_name()] = platform

        # add mesh
        mesh_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), platform.get_platform_mesh_filename())
        FileUtils.copy_file(mesh_filepath, self.get_stimulus_meshes_dirpath())

        # add mjcf
        self._stim_mjcf_generator.add_body(platform.get_platform_name(),
                                           platform.get_platform_mesh_name(),
                                           platform.get_platform_mesh_filename(),
                                           platform_pose.get_position().get_position_as_str(),
                                           platform_pose.get_rotation().get_rotation_as_str(),
                                           platform.get_mass())

        return platform.get_platform_name()

    def add_ground(self, ground_pose=NavPose(NavPosition(0.0, 0.0, -PlatformConfig.PLATFORM_HEIGHT))):
        ground_pose.get_position().add_z(-StimuliConfig.GROUND_HEIGHT)
        ground_pose_str = ground_pose.get_position().get_position_as_str()

        platform = Platform(str(self._stim_platforms_counter), PlatformType.get_ground(), ground_pose)
        ground_size = f"{StimuliConfig.GROUND_LENGTH} {StimuliConfig.GROUND_WIDTH} {StimuliConfig.GROUND_HEIGHT}"
        self._stim_mjcf_generator.add_ground(platform.get_platform_name(), ground_pose_str, ground_size)

    def add_camera(self):
        camera_pos_str = f"{self.get_center_x()} -30 10"
        self._stim_mjcf_generator.add_camera(camera_pos_str, "1.000 -0.016 -0.000 0.005 0.328 0.945")

    def load_stimuli_from_library(self, mjcf_filepath=None):
        if mjcf_filepath is None:
            mjcf_filepath = self._stim_mjcf_generator.get_mjcf_filepath()

        mj = MujocoUtils(mjcf_filepath)
        for platform_name in mj.get_body_names_list():
            platform_pos, platform_rot = mj.get_body_pose_by_name(platform_name)
            platform_pose = NavPose(NavPosition(*platform_pos), NavRotation(*platform_rot))
            self._stim_platforms_dict[platform_name] = Platform.from_str(platform_name, platform_pose)

    def save_to_mjcf(self):
        self._stim_mjcf_generator.save_mjcf()

    def visualize(self):
        mj = MujocoUtils(self._stim_mjcf_generator.get_mjcf_filepath())
        mj.visualize()

    @staticmethod
    def get_root_platform_pose() -> NavPose:
        return NavPose(NavPosition(0.0, 0.0, -PlatformConfig.PLATFORM_HEIGHT))

    def get_next_platform_pose(self, query_platform_type, ref_platform_index, delta_x, delta_y) -> NavPose:
        if ref_platform_index >= len(list(self._stim_platforms_dict.keys())):
            Msg.print_error(f"ERROR [Stimuli]: Platform {ref_platform_index} not found.")
            assert False

        ref_platform_name = self.get_platform_name_by_index(ref_platform_index)

        next_x = self.get_platform(ref_platform_name).get_platform_pose().get_position().get_x()
        next_y = self.get_platform(ref_platform_name).get_platform_pose().get_position().get_y()

        ref_surface_xy = self.get_platform_top_surface_xy(ref_platform_name)
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

    def get_stimulus_mjcf_filepath(self):
        return PathUtils.join(self.get_stimulus_dirpath(), "stimulus.mjcf")

    def get_platform_names_list(self):
        return list(self._stim_platforms_dict.keys())

    def get_platform_name_by_index(self, query_index):
        query_platform_name = ""
        if query_index == -1:
            query_platform_name = self.get_platform_names_list()[-1]

        if query_index < len(self.get_platform_names_list()):
            query_platform_name = self.get_platform_names_list()[query_index]

        return query_platform_name

    def get_platform_by_index(self, query_index):
        query_platform_name = self.get_platform_name_by_index(query_index)
        if self.get_platform_name_by_index(query_index):
            return self.get_platform(query_platform_name)

        Msg.print_error(f"ERROR [Stimuli]: platform index {query_index} not found.")
        assert False

    def get_platforms_list(self):
        return self._stim_platforms_dict.values()

    def get_platform(self, platform_name) -> Platform:
        if platform_name not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_name} not found")
            assert False

        return self._stim_platforms_dict[platform_name]

    def get_platform_center(self, platform_name):
        if platform_name not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_name} not found")
            assert False

        return self.get_platform(platform_name).get_platform_pose().get_position().get_position_as_np_array()

    def get_platform_top_surface_xy(self, platform_name):
        if platform_name not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_name} not found")
            assert False

        return Platform.get_platform_top_surface_xy(self.get_platform(platform_name).get_platform_type())

    def get_point_closest_to_platform(self, ref_platform_name, goal_platform_name):
        if ref_platform_name not in self._stim_platforms_dict or goal_platform_name not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {ref_platform_name} or {goal_platform_name} not found.")

        pos_p1 = self.get_platform(ref_platform_name).get_platform_pose().get_position().get_position_as_np_array()
        pos_p2 = self.get_platform(goal_platform_name).get_platform_pose().get_position().get_position_as_np_array()

        surface_xy_p1 = self.get_platform_top_surface_xy(ref_platform_name)
        surface_xy_p1 = surface_xy_p1[0] / 2, surface_xy_p1[1] / 2

        closest_x = np.clip(pos_p2[0], pos_p1[0] - surface_xy_p1[0], pos_p1[0] + surface_xy_p1[0])
        closest_y = np.clip(pos_p2[1], pos_p1[1] - surface_xy_p1[1], pos_p1[1] + surface_xy_p1[1])

        return np.array([closest_x, closest_y])

    def get_gap_between_platform_centers(self, platform_name1, platform_name2):
        if platform_name1 not in self._stim_platforms_dict or platform_name2 not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_name1} or {platform_name2} not found.")
            assert False

        pos_p1 = self.get_platform(platform_name1).get_platform_pose().get_position().get_position_as_list()
        pos_p2 = self.get_platform(platform_name2).get_platform_pose().get_position().get_position_as_list()
        return ComputeUtils.compute_l2_distance(pos_p1, pos_p2)

    def get_gap_between_platform_edges(self, platform_name1, platform_name2):
        if platform_name1 not in self._stim_platforms_dict or platform_name2 not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_name1} or {platform_name2} not found.")
            assert False

        gap_between_centers = self.get_gap_between_platform_centers(platform_name1, platform_name2)
        if gap_between_centers == 0.0:
            return gap_between_centers

        pos_p2 = self.get_platform(platform_name2).get_platform_pose().get_position().get_position_as_np_array()
        pos_p1 = self.get_platform(platform_name1).get_platform_pose().get_position().get_position_as_np_array()

        surface_xy_p1 = self.get_platform_top_surface_xy(platform_name1)
        surface_xy_p2 = self.get_platform_top_surface_xy(platform_name2)

        dx = abs(pos_p2[0] - pos_p1[0]) - (surface_xy_p1[0] / 2 + surface_xy_p2[0] / 2)
        dy = abs(pos_p2[1] - pos_p1[1]) - (surface_xy_p1[1] / 2 + surface_xy_p2[1] / 2)

        dx = max(dx, 0.0)
        dy = max(dy, 0.0)

        return np.linalg.norm([dx, dy])

    def get_distance_to_platform_center(self, platform_name, ref_pos_array):
        if platform_name not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_name} not found")
            assert False

        ref_pos = self.get_platform(platform_name).get_platform_pose().get_position().get_position_as_np_array()
        return ref_pos[0] - ref_pos_array[0]

    def get_distance_to_platform_edge(self, platform_name, ref_pos_array):
        if platform_name not in self._stim_platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_name} not found")
            assert False

        ref_pos = self.get_platform(platform_name).get_platform_pose().get_position().get_position_as_np_array()
        ref_surface_xy = self.get_platform_top_surface_xy(platform_name)

        return ref_pos[0] + ref_surface_xy[0] / 2 - ref_pos_array[0]

    def get_center_x(self):
        min_platform_list = []
        max_platform_list = []
        for platform_name in self.get_platform_names_list():
            platform_x = self.get_platform(platform_name).get_platform_pose().get_position().get_x()
            surface_xy = self.get_platform_top_surface_xy(platform_name)
            min_platform_list.append(platform_x - surface_xy[0] / 2)
            max_platform_list.append(platform_x + surface_xy[0] / 2)

        min_x = min(min_platform_list)
        max_x = max(max_platform_list)

        return min_x + (max_x - min_x) / 2.0

    def get_max_x(self):
        max_platform_list = []
        for platform_name in self.get_platform_names_list():
            platform_x = self.get_platform(platform_name).get_platform_pose().get_position().get_x()
            surface_xy = self.get_platform_top_surface_xy(platform_name)
            max_platform_list.append(platform_x + surface_xy[0] / 2)
        return max(max_platform_list)
