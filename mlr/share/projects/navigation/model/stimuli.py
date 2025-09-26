from itertools import product

from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavPosition, NavPose
from mlr.share.projects.navigation.utils.platform_utils import PlatformType, Platform
from mlr.share.projects.navigation.utils.stimuli_utils import StimuliSet, StimulusItem


class Counter:
    def __init__(self, start: int = 0):
        self._value = start

    def __call__(self):
        return self._value

    def __iadd__(self, increment: int):
        self._value += increment
        return self


class StimuliDiff(StimuliSet):
    def __init__(self):
        super().__init__("diff")

    def _get_platform_interval(self):
        return

    @staticmethod
    def _get_platform_height():
        return Platform.PLATFORM_HEIGHT / 2.  # scaled down for blender

    @staticmethod
    def _add_platform_pose():
        return NavPose(NavPosition(0.0, 0.0, -StimuliDiff._get_platform_height()))

    @staticmethod
    def _get_next_platform_pose(stim_item: StimulusItem, platform_name):
        prev_key = stim_item.get_platform_key_by_index(-1)

        x = stim_item.get_platform_by_key(prev_key).get_platform_pose().get_position().get_x()

        prev_x, _ = stim_item.get_platform_surface_measures(prev_key)
        next_x, _ = PlatformType.get_platform_surface_measures(platform_name)

        new_loc = x + prev_x / 2.0 + ConfigUtils.STIMULI_PLATFORM_GAP + next_x / 2.0
        return NavPose(NavPosition(new_loc, 0.0, -StimuliDiff._get_platform_height()))

    def _get_stim_item_name(self, stim_counter):
        return self.get_stimuli_set_name() + f"_{stim_counter}"

    def _make_stimuli(self, stim_counter, platform_names_list):
        stim_item = StimulusItem(self._get_stim_item_name(stim_counter()), self)

        stim_item.add_start_platform(NavPose(NavPosition(0.0, 0.0, -StimuliDiff._get_platform_height())))
        for platform_index, platform_name in enumerate(platform_names_list, 1):
            platform_pose = self._get_next_platform_pose(stim_item, platform_name)
            platform_filename = platform_name + ".stl"
            stim_item.add_platform(platform_index, platform_pose, platform_filename)
        stim_item.add_final_platform(self._get_next_platform_pose(stim_item, "final_final"))

        stim_center_x = stim_item.get_stim_item_center_x()
        stim_item.add_ground_plane(NavPose(NavPosition(stim_center_x, 0.0, -StimuliDiff._get_platform_height() * 2)))

        stim_counter += 1

    def _make_special_set(self, stim_counter, special_shape, special_scale):
        platform_list = [PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(special_shape, special_scale),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED)]
        self._make_stimuli(stim_counter, platform_list)

        platform_list = [PlatformType.get_platform_name(special_shape, special_scale),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED)]
        self._make_stimuli(stim_counter, platform_list)

        platform_list = [PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(special_shape, special_scale)]
        self._make_stimuli(stim_counter, platform_list)

        platform_list = [PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(special_shape, special_scale),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED)]
        self._make_stimuli(stim_counter, platform_list)

        platform_list = [PlatformType.get_platform_name(special_shape, special_scale),
                         PlatformType.get_platform_name(special_shape, special_scale),
                         PlatformType.get_platform_name(special_shape, special_scale)]
        self._make_stimuli(stim_counter, platform_list)

    def make_stimuli_01(self, stim_counter):
        platform_list = [PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED),
                         PlatformType.get_platform_name(PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED)]
        self._make_stimuli(stim_counter, platform_list)

    def make_stimuli_02(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_CUBE, PlatformType.NOT_SCALED)

    def make_stimuli_03(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_CUBE, PlatformType.TOP_SCALED)

    def make_stimuli_04(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_CUBE, PlatformType.BOT_SCALED)

    def make_stimuli_05(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_WIDE, PlatformType.NOT_SCALED)

    def make_stimuli_06(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_WIDE, PlatformType.TOP_SCALED)

    def make_stimuli_07(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_WIDE, PlatformType.BOT_SCALED)

    def make_stimuli_08(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_LONG, PlatformType.NOT_SCALED)

    def make_stimuli_09(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_LONG, PlatformType.TOP_SCALED)

    def make_stimuli_10(self, stim_counter):
        self._make_special_set(stim_counter, PlatformType.CUBOIDAL_LONG, PlatformType.BOT_SCALED)


class StimuliPairs(StimuliSet):
    def __init__(self):
        super().__init__("pairs")

    def make_platform_pairs(self):
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

        for (p_start_name, p_final_name) in list(product(StimuliSet.get_available_platform_names_list(), repeat=2)):
            stim_name = p_start_name + "_" + p_final_name
            platform_start_filename = p_start_name + ".stl"
            platform_final_filename = p_final_name + ".stl"

            stim_item = StimulusItem(stim_name, self)
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
    # StimuliPairs().make_platform_pairs()

    stim_counter = Counter(1)
    StimuliDiff().make_stimuli_01(stim_counter)
    StimuliDiff().make_stimuli_02(stim_counter)
    StimuliDiff().make_stimuli_03(stim_counter)
    StimuliDiff().make_stimuli_04(stim_counter)
    StimuliDiff().make_stimuli_05(stim_counter)
    StimuliDiff().make_stimuli_06(stim_counter)
    StimuliDiff().make_stimuli_07(stim_counter)
    StimuliDiff().make_stimuli_08(stim_counter)
    StimuliDiff().make_stimuli_09(stim_counter)
    StimuliDiff().make_stimuli_10(stim_counter)


if __name__ == '__main__':
    main()
