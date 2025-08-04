from itertools import product

from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavPosition, NavPose
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.platform_utils import PlatformType, Platform
from mlr.share.projects.navigation.utils.stimuli_utils import StimuliSet, StimulusItem


class StimuliPairs(StimuliSet):
    def __init__(self):
        super().__init__("pairs")

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

            stim_item = StimulusItem(stim_name,StimuliSet("pairs"))
            stim_item.add_ground_plane(ground_pose)
            stim_item.add_platform(1, start_pose, platform_start_filename)

            is_first_wide = p_start_name.startswith(PlatformType.CUBOIDAL_WIDE)
            is_second_wide = p_final_name.startswith(PlatformType.CUBOIDAL_WIDE)
            if is_first_wide and is_second_wide:
                stim_item.add_platform(2, wide_wide_final_pose, platform_final_filename)
            elif is_first_wide or is_second_wide:
                stim_item.add_platform(2, wide_std_final_pose, platform_final_filename)
            else:
                stim_item.add_platform(2, final_pose, platform_final_filename)


def main():
    StimuliPairs.make_platform_pairs()


if __name__ == '__main__':
    main()
