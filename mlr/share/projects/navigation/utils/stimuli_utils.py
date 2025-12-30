import numpy as np

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.navigation_utils import NavPose, NavPosition, NavRotation
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.platform_utils import Platform, PlatformType
from mlr.share.projects.navigation.utils.urdf_utils import URDFGenerator


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
    def get_available_platform_names_list():
        platforms_dirpath = PathUtils.get_platforms_dirpath()
        platform_names = FileUtils.get_files_in_directory(platforms_dirpath)
        return [FileUtils.get_basename(name).split(".")[0] for name in platform_names]


class StimulusItem:
    def __init__(self, stimulus_item_name, stimuli_set: StimuliSet):
        self._stimulus_item_name = stimulus_item_name
        self._stimuli_set = stimuli_set

        self._platforms_dict = {}

        self._prepare_directories()

        self._root_urdf_generator = URDFGenerator("stim", self.get_urdf_dirpath())

    def _prepare_directories(self):
        FileUtils.create_dir(self.get_stimulus_item_dirpath())
        FileUtils.create_dir(self.get_urdf_dirpath())
        FileUtils.create_dir(self.get_meshes_dirpath())

    def _add_element(self, element_index, element_name, element_pose, element_mass, element_mesh_filename=None):
        platform_key = self._get_element_key(element_index, element_name, element_mesh_filename)
        platform_urdf_filename = self._get_element_key(element_index, element_name)
        platform = Platform(platform_key, element_pose)

        self._platforms_dict[platform_key] = platform

        # add mesh
        if element_mesh_filename.endswith(".obj"):
            element_mesh_filepath = PathUtils.join(PathUtils.get_misc_dirpath(), element_mesh_filename)
        else:
            element_mesh_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), element_mesh_filename)

        FileUtils.copy_file(element_mesh_filepath, self.get_meshes_dirpath())

        # add URDF
        urdf_generator = URDFGenerator(platform_urdf_filename, self.get_urdf_dirpath())
        urdf_generator.add_element(platform_key,
                                   element_pose.get_position().get_position_as_str(),
                                   element_pose.get_rotation().get_rotation_as_str(),
                                   element_mass, element_mesh_filename)
        urdf_generator.save_urdf()

        self._root_urdf_generator.add_element(platform_key,
                                   element_pose.get_position().get_position_as_str(),
                                   element_pose.get_rotation().get_rotation_as_str(),
                                   element_mass, element_mesh_filename)
        self._root_urdf_generator.save_urdf()
        self._root_urdf_generator.save_as_stl()
        return platform

    @staticmethod
    def _get_urdf_filename(platform_key, with_extension=True):
        filename = platform_key.split("_")[0]
        if with_extension:
            return filename + ".urdf"
        return filename

    @staticmethod
    def _get_element_key(element_index, element_name, element_mesh_filename=None):
        if element_mesh_filename is None:
            return element_name + str(element_index)
        return element_name + str(element_index) + "_" + element_mesh_filename.split(".")[0]

    def add_ground_plane(self, ground_pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        self._add_element(1, URDFGenerator.ELEMENT_GROUND, ground_pose, 1.0, URDFGenerator.ELEMENT_GROUND_OBJ_FILENAME)

    def add_start_platform(self, pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        self._add_element(1, URDFGenerator.ELEMENT_START, pose, 1.0, URDFGenerator.ELEMENT_START_OBJ_FILENAME)

    def add_final_platform(self, pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        self._add_element(1, URDFGenerator.ELEMENT_FINAL, pose, 1.0, URDFGenerator.ELEMENT_FINAL_OBJ_FILENAME)

    def add_platform(self, platform_index, platform_pose, platform_filename):
        self._add_element(platform_index,
                          URDFGenerator.ELEMENT_PLATFORM,
                          platform_pose,
                          ConfigUtils.STIMULI_PLATFORM_MASS,
                          platform_filename)

    def load_stimuli_from_library(self, urdf_dirpath=None):
        if urdf_dirpath is None:
            urdf_dirpath = self.get_urdf_dirpath()

        for urdf_filepath in FileUtils.get_files_in_directory(urdf_dirpath):
            element_name = FileUtils.get_basename(urdf_filepath, is_attached=False)[0]
            if element_name == URDFGenerator.ELEMENT_GROUND:
                continue

            if element_name == "stim":
                continue

            platform_key = URDFGenerator.get_element_name(urdf_filepath)
            platform_pos, platform_rot = URDFGenerator.get_element_pos_rot_list(urdf_filepath)
            platform_pose = NavPose(NavPosition(*platform_pos), NavRotation(*platform_rot))
            self._platforms_dict[platform_key] = Platform(platform_key, platform_pose)

    @staticmethod
    def is_platform_movable(platform_key):
        if platform_key.startswith(URDFGenerator.ELEMENT_PLATFORM):
            return True
        return False

    def get_stimulus_item_name(self, basename_only=False):
        if basename_only:
            return FileUtils.get_basename(self._stimulus_item_name)
        return self._stimulus_item_name

    def get_stimulus_item_dirpath(self):
        return PathUtils.join(self._stimuli_set.get_stimuli_set_dirpath(), self._stimulus_item_name)

    def get_urdf_dirpath(self):
        return PathUtils.join(self.get_stimulus_item_dirpath(), "urdf")

    def get_urdf_filepath(self, platform_key):
        return URDFGenerator.get_urdf_filepath(self.get_urdf_dirpath(), self._get_urdf_filename(platform_key, False))

    def get_urdf_rel_filepath(self, platform_key):
        return PathUtils.get_relative_path(self.get_urdf_filepath(platform_key), self.get_stimulus_item_dirpath())

    def get_meshes_dirpath(self):
        return PathUtils.join(self.get_stimulus_item_dirpath(), "meshes")

    def get_platform_keys_list(self, exclude_ground=True):
        return_list = list(sorted(self._platforms_dict.keys()))
        if exclude_ground:
            return [p for p in return_list if not p.startswith(URDFGenerator.ELEMENT_GROUND)]
        return return_list

    def get_platform_ground_key(self):
        return list(set(self.get_platform_keys_list(False)) - set(self.get_platform_keys_list(True)))[0]

    def get_platform_key_by_index(self, query_index):
        if query_index == -1:
            return self.get_platform_keys_list()[-1]

        for platform_index, platform_key in enumerate(self.get_platform_keys_list()):
            if platform_index == query_index:
                return platform_key
        Msg.print_error(f"ERROR [Stimuli]: platform index {query_index} not found.")
        assert False

    def get_platform_by_key(self, platform_key):
        if platform_key not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: platform {platform_key} not found.")
            assert False

        return self._platforms_dict[platform_key]

    def get_platform_surface_measures(self, platform_key):
        if platform_key not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {platform_key} not found.")
            assert False

        return PlatformType.get_platform_surface_measures("_".join(platform_key.split("_")[1:]))

    def get_gap_between_platforms(self, platform_key1, platform_key2):
        if platform_key1 not in self._platforms_dict or platform_key2 not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_key1} or {platform_key2} not found.")
            assert False

        p1 = self._platforms_dict[platform_key1].get_platform_pose().get_position().get_position_as_list()
        p2 = self._platforms_dict[platform_key2].get_platform_pose().get_position().get_position_as_list()

        return ComputeUtils.compute_l2_distance(p1, p2)

    def get_gap_to_next_platform(self, platform_key):
        if platform_key not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {platform_key} not found.")
            assert False

        platform_x, _ = self.get_platform_surface_measures(platform_key)

        return platform_x / 2.0 + ConfigUtils.STIMULI_PLATFORM_GAP

    def get_stim_item_center_x(self):
        min_x = np.inf
        max_x = -np.inf

        platform_key_list = self.get_platform_keys_list(exclude_ground=True)
        for platform_key in platform_key_list:
            platform = self.get_platform_by_key(platform_key)
            platform_x_pos = platform.get_platform_pose().get_position().get_x()
            if platform_x_pos < min_x:
                min_x = platform_x_pos
            if platform_x_pos > max_x:
                max_x = platform_x_pos

        return min_x + (max_x - min_x) / 2.0
