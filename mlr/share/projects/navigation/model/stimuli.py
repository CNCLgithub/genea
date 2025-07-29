from itertools import product

from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.navigation_utils import NavPose, NavPosition, NavRotation
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.platform_utils import PlatformType, Platform
from mlr.share.projects.navigation.utils.urdf_utils import URDFGenerator


class Stimuli:
    ROOT_P = "p"
    ROOT_G = "g"

    GROUND_OBJ_FILENAME = "ground_plane.obj"

    def __init__(self, stimuli_dirpath=None):
        self._stimuli_dirpath = stimuli_dirpath

        self._platforms_dict = {}

    def get_stimuli_dirpath(self):
        return self._stimuli_dirpath

    def get_stimuli_urdf_dirpath(self):
        return PathUtils.get_stimuli_urdf_dirpath(self._stimuli_dirpath)

    def get_stimuli_name(self):
        return FileUtils.get_basename(self._stimuli_dirpath)

    def get_ground_urdf_filepath(self):
        ground_filepath = URDFGenerator.get_urdf_filepath(self.get_stimuli_urdf_dirpath(), Stimuli.ROOT_G)
        ground_rel_filepath = PathUtils.get_relative_path(ground_filepath, self.get_stimuli_dirpath())
        return ground_filepath, ground_rel_filepath

    def get_platform_urdf_filepath(self, platform_name):
        platform_filepath = URDFGenerator.get_urdf_filepath(self.get_stimuli_urdf_dirpath(), platform_name)
        platform_rel_filepath = PathUtils.get_relative_path(platform_filepath, self.get_stimuli_dirpath())
        return platform_filepath, platform_rel_filepath

    def get_platform_names_list(self):
        return list(self._platforms_dict.keys())

    def _add_platform(self, platform_name, platform: Platform):
        if platform not in self._platforms_dict:
            self._platforms_dict[platform_name] = platform
        self._platforms_dict[platform_name] = platform

    def load_stimuli_from_library(self, urdf_dirpath=None):
        if urdf_dirpath is None:
            urdf_dirpath = self.get_stimuli_urdf_dirpath()

        for urdf_filepath in FileUtils.get_files_in_directory(urdf_dirpath):
            object_name = FileUtils.get_basename(urdf_filepath, is_attached=False)[0]
            if object_name == Stimuli.ROOT_G:
                continue

            platform_pos, platform_rot = URDFGenerator.get_object_pos_rot_list(urdf_filepath)
            platform_pose = NavPose(NavPosition(*platform_pos), NavRotation(*platform_rot))
            platform = Platform(object_name, platform_pose)
            self._add_platform(object_name, platform)

    def get_gap_between_platforms(self, platform_name1, platform_name2):
        if platform_name1 not in self._platforms_dict or platform_name2 not in self._platforms_dict:
            Msg.print_error(f"ERROR [Stimuli]: Platforms {platform_name1} or {platform_name2} not found.")
            assert False

        p1 = self._platforms_dict[platform_name1].get_platform_pose().get_position().get_position_as_list()
        p2 = self._platforms_dict[platform_name2].get_platform_pose().get_position().get_position_as_list()

        return ComputeUtils.compute_l2_distance(p1, p2)


class StimuliPairs(Stimuli):
    @staticmethod
    def _get_available_platform_names_list():
        platforms_dirpath = PathUtils.get_platforms_dirpath()
        platform_names = FileUtils.get_files_in_directory(platforms_dirpath)
        return [FileUtils.get_basename(name).split(".")[0] for name in platform_names]

    @staticmethod
    def make_platform_pairs():
        std_half_size = Platform.PLATFORM_SIZE_NORMAL / 2.0
        wide_half_size = Platform.PLATFORM_SIZE_SCALED / 2.0

        final_loc = std_half_size + ConfigUtils.STIMULI_PLATFORM_GAP + std_half_size
        wide_std_final_loc = wide_half_size + ConfigUtils.STIMULI_PLATFORM_GAP + std_half_size
        wide_wide_final_loc = wide_half_size + ConfigUtils.STIMULI_PLATFORM_GAP + wide_half_size

        ground_pose = NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT))
        start_pose = NavPose(NavPosition(0.0, 0.0, -Platform.PLATFORM_HEIGHT / 2.0))

        final_pose = NavPose(NavPosition(final_loc, 0.0, -Platform.PLATFORM_HEIGHT / 2.0))
        wide_std_final_pose = NavPose(NavPosition(wide_std_final_loc, 0.0, -Platform.PLATFORM_HEIGHT / 2.0))
        wide_wide_final_pose = NavPose(NavPosition(wide_wide_final_loc, 0.0, -Platform.PLATFORM_HEIGHT / 2.0))

        for (p_start_name, p_final_name) in list(product(StimuliPairs._get_available_platform_names_list(), repeat=2)):
            stim_name = p_start_name + "_" + p_final_name
            platform_start_filename = p_start_name + ".stl"
            platform_final_filename = p_final_name + ".stl"

            FileUtils.create_dir(PathUtils.get_stimuli_pairs_dirpath(stim_name))
            FileUtils.create_dir(PathUtils.get_stimuli_pairs_meshes_dirpath(stim_name))
            FileUtils.create_dir(PathUtils.get_stimuli_pairs_urdf_dirpath(stim_name))

            ground_plane_filepath = PathUtils.join(PathUtils.get_misc_dirpath(), Stimuli.GROUND_OBJ_FILENAME)
            platform_start_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), platform_start_filename)
            platform_final_filepath = PathUtils.join(PathUtils.get_platforms_dirpath(), platform_final_filename)

            FileUtils.copy_file(ground_plane_filepath, PathUtils.get_stimuli_pairs_meshes_dirpath(stim_name))
            FileUtils.copy_file(platform_start_filepath, PathUtils.get_stimuli_pairs_meshes_dirpath(stim_name))
            FileUtils.copy_file(platform_final_filepath, PathUtils.get_stimuli_pairs_meshes_dirpath(stim_name))

            # ----------------- make ground plane -----------------
            urdf_generator = URDFGenerator(Stimuli.ROOT_G, PathUtils.get_stimuli_pairs_urdf_dirpath(stim_name))
            urdf_generator.add_object("ground",
                                      ground_pose.get_position().get_position_as_str(),
                                      ground_pose.get_rotation().get_rotation_as_str(),
                                      1.0, Stimuli.GROUND_OBJ_FILENAME)
            urdf_generator.save_urdf()

            # ----------------- make platform 1 -----------------
            urdf_generator = URDFGenerator(Stimuli.ROOT_P + str(1), PathUtils.get_stimuli_pairs_urdf_dirpath(stim_name))
            urdf_generator.add_object(Stimuli.ROOT_P + str(1) + "_" + p_start_name,
                                      start_pose.get_position().get_position_as_str(),
                                      start_pose.get_rotation().get_rotation_as_str(),
                                      Platform.PLATFORM_MASS, platform_start_filename)
            urdf_generator.save_urdf()

            # ----------------- make platform 2 -----------------
            urdf_generator = URDFGenerator(Stimuli.ROOT_P + str(2), PathUtils.get_stimuli_pairs_urdf_dirpath(stim_name))

            is_first_wide = p_start_name.startswith(PlatformType.CUBOIDAL_WIDE)
            is_second_wide = p_final_name.startswith(PlatformType.CUBOIDAL_WIDE)
            if is_first_wide and is_second_wide:
                urdf_generator.add_object(Stimuli.ROOT_P + str(2) + "_" + p_final_name,
                                          wide_wide_final_pose.get_position().get_position_as_str(),
                                          wide_wide_final_pose.get_rotation().get_rotation_as_str(),
                                          Platform.PLATFORM_MASS, platform_final_filename)
            elif is_first_wide or is_second_wide:
                urdf_generator.add_object(Stimuli.ROOT_P + str(2) + "_" + p_final_name,
                                          wide_std_final_pose.get_position().get_position_as_str(),
                                          wide_std_final_pose.get_rotation().get_rotation_as_str(),
                                          Platform.PLATFORM_MASS, platform_final_filename)
            else:
                urdf_generator.add_object(Stimuli.ROOT_P + str(2) + "_" + p_final_name,
                                          final_pose.get_position().get_position_as_str(),
                                          final_pose.get_rotation().get_rotation_as_str(),
                                          Platform.PLATFORM_MASS, platform_final_filename)
            urdf_generator.save_urdf()


def main():
    StimuliPairs.make_platform_pairs()


if __name__ == '__main__':
    main()
