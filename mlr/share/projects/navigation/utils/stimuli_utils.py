from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.navigation_utils import NavPose, NavPosition, NavRotation
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.platform_utils import Platform
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


class StimulusItem:
    ELEMENT_PLATFORM = "p"
    ELEMENT_GROUND = "g"

    ELEMENT_GROUND_OBJ_FILENAME = "ground_plane.obj"

    def __init__(self, stimulus_item_name, stimuli_set: StimuliSet):
        self._stimulus_item_name = stimulus_item_name
        self._stimuli_set = stimuli_set

        self._platforms_dict = {}

        self._prepare_directories()

    def _prepare_directories(self):
        FileUtils.create_dir(self.get_stimulus_item_dirpath())
        FileUtils.create_dir(self.get_urdf_dirpath())
        FileUtils.create_dir(self.get_meshes_dirpath())

    def get_stimulus_item_name(self):
        return self._stimulus_item_name
    
    def get_stimulus_item_dirpath(self):
        return PathUtils.join(self._stimuli_set.get_stimuli_set_dirpath(), self._stimulus_item_name)

    def get_urdf_dirpath(self):
        return PathUtils.join(self.get_stimulus_item_dirpath(), "urdf")
    
    def get_urdf_filepath(self, platform_name):
        return URDFGenerator.get_urdf_filepath(self.get_urdf_dirpath(), platform_name)

    def get_urdf_rel_filepath(self, platform_name):
        return PathUtils.get_relative_path(self.get_urdf_filepath(platform_name), self.get_stimulus_item_dirpath())

    def get_meshes_dirpath(self):
        return PathUtils.join(self.get_stimulus_item_dirpath(), "meshes")

    def get_platform_names_list(self):
        return list(self._platforms_dict.keys())

    def get_platform_by_name(self, platform_name):
        if platform_name not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platform {platform_name} not found.")
            assert False

        return self._platforms_dict[platform_name]

    @staticmethod
    def get_idx_element_name(element_index, element_name, element_mesh_filename=None):
        if element_mesh_filename is None:
            return element_name + str(element_index)
        return element_name + str(element_index) + "_" + element_mesh_filename.split(".")[0]

    def _add_element(self, element_index, element_name, element_pose, element_mass, element_mesh_filename=None):
        if element_mesh_filename == StimulusItem.ELEMENT_GROUND_OBJ_FILENAME:
            element_mesh_filepath = PathUtils.join(PathUtils.get_misc_dirpath(), element_mesh_filename)
        else:
            element_mesh_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), element_mesh_filename)

        # add mesh
        FileUtils.copy_file(element_mesh_filepath, self.get_meshes_dirpath())

        # add URDF
        urdf_generator = URDFGenerator(self.get_idx_element_name(element_index, element_name), self.get_urdf_dirpath())
        urdf_generator.add_element(self.get_idx_element_name(element_index, element_name, element_mesh_filename),
                                   element_pose.get_position().get_position_as_str(),
                                   element_pose.get_rotation().get_rotation_as_str(),
                                   element_mass, element_mesh_filename)
        urdf_generator.save_urdf()

    def add_ground_plane(self, ground_pose=NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))):
        self._add_element(1, StimulusItem.ELEMENT_GROUND, ground_pose, 1.0, StimulusItem.ELEMENT_GROUND_OBJ_FILENAME)

    def add_platform(self, platform_index, platform_pose, platform_filename):
        self._add_element(platform_index,
                          StimulusItem.ELEMENT_PLATFORM,
                          platform_pose,
                          ConfigUtils.STIMULI_PLATFORM_MASS,
                          platform_filename)

    def load_stimuli_from_library(self, urdf_dirpath=None):
        if urdf_dirpath is None:
            urdf_dirpath = self.get_urdf_dirpath()

        for urdf_filepath in FileUtils.get_files_in_directory(urdf_dirpath):
            element_name = FileUtils.get_basename(urdf_filepath, is_attached=False)[0]
            if element_name == StimulusItem.ELEMENT_GROUND:
                continue

            platform_name = URDFGenerator.get_element_name(urdf_filepath)

            platform_pos, platform_rot = URDFGenerator.get_element_pos_rot_list(urdf_filepath)
            platform_pose = NavPose(NavPosition(*platform_pos), NavRotation(*platform_rot))
            platform = Platform(platform_name, platform_pose)

            if platform not in self._platforms_dict:
                self._platforms_dict[platform_name] = platform
            self._platforms_dict[platform_name] = platform

    def get_gap_between_platforms(self, platform_name1, platform_name2):
        if platform_name1 not in self._platforms_dict or platform_name2 not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_name1} or {platform_name2} not found.")
            assert False

        p1 = self._platforms_dict[platform_name1].get_platform_pose().get_position().get_position_as_list()
        p2 = self._platforms_dict[platform_name2].get_platform_pose().get_position().get_position_as_list()

        return ComputeUtils.compute_l2_distance(p1, p2)
