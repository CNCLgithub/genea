from mlr.share.projects.navigation.utils.config_utils import PlatformConfig, StimuliConfig
from mlr.share.projects.navigation.utils.core_utils import NavPosition, NavPose
from mlr.share.projects.navigation.utils.platform_utils import PlatformType, PlatformShape, PlatformScale, \
    PlatformMaterial
from mlr.share.projects.navigation.utils.stimuli_utils import StimuliSet, Stimulus


class StimuliTestWalk(StimuliSet):
    def __init__(self):
        super().__init__("test_walk")

    def make_stimuli(self):
        platform_type = PlatformType(PlatformShape.WALK, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY)

        stimulus = Stimulus(self)
        stimulus.add_platform(platform_type, NavPose(NavPosition(0.0, 0.0, -PlatformConfig.PLATFORM_HEIGHT)))
        stimulus.add_ground(NavPose(NavPosition(stimulus.get_stim_center_x(), 0.0, -PlatformConfig.PLATFORM_HEIGHT)))
        stimulus.add_camera()
        stimulus.save_to_mjcf()
        stimulus.visualize()


class StimuliTestJump(StimuliSet):
    def __init__(self):
        super().__init__("test_jump")

    def make_stimuli(self):
        platform_type1 = PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY)
        platform_type2 = PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)

        stimulus = Stimulus(self)
        stimulus.add_platform(platform_type1, NavPose(NavPosition(0.0, 0.0, -PlatformConfig.PLATFORM_HEIGHT)))
        stimulus.add_platform(platform_type2, stimulus.get_next_platform_pose(platform_type2, 0, 0.0, 0.0))
        stimulus.add_ground(NavPose(NavPosition(stimulus.get_stim_center_x(), 0.0, -PlatformConfig.PLATFORM_HEIGHT)))
        stimulus.add_camera()
        stimulus.save_to_mjcf()
        stimulus.visualize()


class StimuliDiff(StimuliSet):
    def __init__(self):
        super().__init__("diff")

    def _make_stimuli(self, platform_types_list, parent_ids_list, dx_list, dy_list, dx_goal=0., dy_goal=0., var=False):
        stimulus = Stimulus(self, is_variant=var)

        stimulus.add_platform(PlatformType.get_root(), stimulus.get_root_platform_pose())
        for platform_type, parent_id, dx, dy in zip(platform_types_list, parent_ids_list, dx_list, dy_list):
            platform_pose = stimulus.get_next_platform_pose(platform_type, parent_id, dx, dy)
            stimulus.add_platform(platform_type, platform_pose)
        stimulus.add_platform(PlatformType.get_goal(), stimulus.get_goal_platform_pose(dx_goal, dy_goal))

        stimulus.add_ground(NavPose(NavPosition(stimulus.get_stim_center_x(), 0.0, -PlatformConfig.PLATFORM_HEIGHT)))
        stimulus.add_camera()
        stimulus.save_to_mjcf()
        stimulus.visualize()

    def _make_permuted_stim_set(self, platform_types_list, parent_ids_list, delta_x_list, delta_y_list):
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

        platform_types_list = [platform_types_list[-1]] + platform_types_list[:-1]
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, var=True)

        platform_types_list = [platform_types_list[-1]] + platform_types_list[:-1]
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, var=True)

    # ======================= BASE =======================
    def make_stimuli_00(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_01(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_02(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE)]

        parent_ids_list = [parent_id for parent_id in [0, 1, 0, 0, 4, 5, 4]]
        offset_x1 = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        offset_x2 = -StimuliConfig.MIN_GAP_X + 1.75
        offset_x3 = -StimuliConfig.MIN_GAP_X + 6.75
        delta_x_list = [offset_x1, offset_x1, offset_x2, offset_x3, offset_x1, offset_x1, offset_x2]
        delta_y_list = [-1.5, -0.0, 1.75, 0.0, 1.5, 0.0, -1.75]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x1, 0.0)

    # ======================= WIDE =======================
    def make_stimuli_03(self):
        platform_types_list = [PlatformType(PlatformShape.WIDE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.WIDE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_04(self):
        platform_types_list = [PlatformType(PlatformShape.WIDE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.WIDE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_05(self):
        platform_types_list = [PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_06(self):
        platform_types_list = [PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    # ======================= LONG =======================
    def make_stimuli_07(self):
        platform_types_list = [PlatformType(PlatformShape.LONG, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.LONG, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_08(self):
        platform_types_list = [PlatformType(PlatformShape.LONG, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.LONG, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_09(self):
        platform_types_list = [PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_10(self):
        platform_types_list = [PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    # ======================= WALK =======================
    def make_stimuli_11(self):
        platform_types_list = [PlatformType(PlatformShape.WALK, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]

        parent_ids_list = [parent_id for parent_id in [0, 0]]
        offset_x = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        delta_x_list = [offset_x, offset_x]
        delta_y_list = [2.0, -2.0]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x, 0.0)

    def make_stimuli_12(self):
        platform_types_list = [PlatformType(PlatformShape.WALK, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in [0, 0, 2]]
        offset_x1 = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        offset_x2 = -StimuliConfig.MIN_GAP_X + 0.83
        delta_x_list = [offset_x1, offset_x2, offset_x2]
        delta_y_list = [2.0, -2.0, 0.0]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x1, 0.0)

    def make_stimuli_13(self):
        platform_types_list = [PlatformType(PlatformShape.WALK, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in [0, 0, 2, 3]]
        offset_x1 = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        offset_x2 = -StimuliConfig.MIN_GAP_X + 0.625
        delta_x_list = [offset_x1, offset_x2, offset_x2, offset_x2]
        delta_y_list = [2.0, -2.0, 0.0, 0.0]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x1, 0.0)

    # ======================= TOP =======================
    def make_stimuli_14(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.TOP_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.TOP_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    def make_stimuli_15(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.TOP_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.TOP_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [0.0 for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_permuted_stim_set(platform_types_list, parent_ids_list, delta_x_list, delta_y_list)

    # ======================= MISC =======================
    def make_stimuli_16(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        offset_x1 = -StimuliConfig.MIN_GAP_X + 1
        offset_x2 = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        delta_x_list = [offset_x1, offset_x2, offset_x2, offset_x2]
        delta_y_list = [2.0, -4.0, 4.0]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x1, 0.0)

    def make_stimuli_17(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        offset_x1 = -StimuliConfig.MIN_GAP_X + 1
        offset_x2 = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        delta_x_list = [offset_x1, offset_x2, offset_x2, offset_x2]
        delta_y_list = [2.0, -4.0, 4.0]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x1, 0.0)

    def make_stimuli_18(self):
        platform_types_list = [PlatformType(PlatformShape.WALK, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        offset_x1 = -StimuliConfig.MIN_GAP_X + 1.5
        offset_x2 = -StimuliConfig.MIN_GAP_X + 0.25
        delta_x_list = [offset_x1, offset_x1]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x2, 0.0)

    def make_stimuli_19(self):
        platform_types_list = [PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        offset_x = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        delta_x_list = [offset_x for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x, 0.0)

    def make_stimuli_20(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY)]

        parent_ids_list = [parent_id for parent_id in [0, 1, 0, 0, 4, 5, 4]]
        offset_x1 = -StimuliConfig.MIN_GAP_X + StimuliConfig.WALK_GAP_X
        offset_x2 = -StimuliConfig.MIN_GAP_X + 1.75
        offset_x3 = -StimuliConfig.MIN_GAP_X + 6.75
        delta_x_list = [offset_x1, offset_x1, offset_x2, offset_x3, offset_x1, offset_x1, offset_x2]
        delta_y_list = [-1.5, -0.0, 1.75, 0.0, 1.5, 0.0, -1.75]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x1, 0.0)

    def make_stimuli_21(self):
        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.WOODY),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE),
                               PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]

        parent_ids_list = [parent_id for parent_id in [0, 1, 2, 3, 0, 0]]
        offset_x1 = -StimuliConfig.MIN_GAP_X + 2.7
        offset_x2 = -StimuliConfig.MIN_GAP_X + 11.5
        delta_x_list = [0.0, 0.0, 0.0, 0.0, offset_x1, offset_x2]
        delta_y_list = [0.0, 0.0, 0.0, 0.0, 3.5, -3.5]

        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, 0.0, 0.0)

    def make_stimuli_22(self):
        offset_x = -StimuliConfig.MIN_GAP_X + 2.

        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [offset_x for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x)

        platform_types_list = [PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.WOODY)]
        parent_ids_list = [parent_id for parent_id in range(len(platform_types_list))]
        delta_x_list = [offset_x for _ in range(len(platform_types_list))]
        delta_y_list = [0.0 for _ in range(len(platform_types_list))]
        self._make_stimuli(platform_types_list, parent_ids_list, delta_x_list, delta_y_list, offset_x, 0.0, var=True)


def main():
    stimuli_diff = StimuliDiff()
    stimuli_diff.make_stimuli_00()
    stimuli_diff.make_stimuli_01()
    stimuli_diff.make_stimuli_02()
    stimuli_diff.make_stimuli_03()
    stimuli_diff.make_stimuli_04()
    stimuli_diff.make_stimuli_05()
    stimuli_diff.make_stimuli_06()
    stimuli_diff.make_stimuli_07()
    stimuli_diff.make_stimuli_08()
    stimuli_diff.make_stimuli_09()
    stimuli_diff.make_stimuli_10()
    stimuli_diff.make_stimuli_11()
    stimuli_diff.make_stimuli_12()
    stimuli_diff.make_stimuli_13()
    stimuli_diff.make_stimuli_14()
    stimuli_diff.make_stimuli_15()
    stimuli_diff.make_stimuli_16()
    stimuli_diff.make_stimuli_17()
    stimuli_diff.make_stimuli_18()
    stimuli_diff.make_stimuli_19()
    stimuli_diff.make_stimuli_20()
    stimuli_diff.make_stimuli_21()
    stimuli_diff.make_stimuli_22()


if __name__ == '__main__':
    main()
