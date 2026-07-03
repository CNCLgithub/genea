from mlr.share.projects.navigation.utils.config_utils import PlatformConfig
from mlr.share.projects.navigation.utils.core_utils import NavPose


class PlatformShape:
    BASE = "base"
    WIDE = "wide"
    LONG = "long"
    WALK = "walk"
    ROOT = "root"
    GOAL = "goal"
    GROUND = "ground"


class PlatformScale:
    NOT_SCALED = "non"
    TOP_SCALED = "top"
    BOT_SCALED = "bot"


class PlatformMaterial:
    WOODY = "woody"
    STONE = "stone"
    BLAND = "bland"
    BRAWN = "brawn"
    BROWN = "brown"


class PlatformType:
    def __init__(self, platform_shape, platform_scale, platform_material):
        self._platform_shape = platform_shape
        self._platform_scale = platform_scale
        self._platform_material = platform_material

    @staticmethod
    def from_str(platform_type_str):
        platform_shape, platform_scale, platform_material = platform_type_str.split("_")
        return PlatformType(platform_shape, platform_scale, platform_material)

    def is_top_scaled(self):
        return self.get_scale() == PlatformScale.TOP_SCALED

    def is_bot_scaled(self):
        return self.get_scale() == PlatformScale.BOT_SCALED

    def is_woody(self):
        return self.get_material() == PlatformMaterial.WOODY

    def is_stone(self):
        return self.get_material() == PlatformMaterial.STONE

    def is_bland(self):
        return self.get_material() == PlatformMaterial.BLAND

    def is_brawn(self):
        return self.get_material() == PlatformMaterial.BRAWN

    def get_shape(self):
        return self._platform_shape

    def get_scale(self):
        return self._platform_scale

    def get_material(self):
        return self._platform_material

    @staticmethod
    def get_base():
        return PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)

    @staticmethod
    def get_ground():
        return PlatformType(PlatformShape.GROUND, PlatformScale.NOT_SCALED, PlatformMaterial.BRAWN)

    @staticmethod
    def get_root():
        return PlatformType(PlatformShape.ROOT, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)

    @staticmethod
    def get_goal():
        return PlatformType(PlatformShape.GOAL, PlatformScale.NOT_SCALED, PlatformMaterial.STONE)


class Platform:
    def __init__(self, platform_id: str, platform_type: PlatformType, platform_pose: NavPose):
        self._platform_id = platform_id
        self._platform_type = platform_type
        self._platform_pose = platform_pose

        self._has_been_visited = False

    @staticmethod
    def from_str(platform_name, platform_pose: NavPose = NavPose.neutral()):
        platform_shape, platform_scale, platform_material, platform_id = platform_name.split("_")
        return Platform(platform_id, PlatformType(platform_shape, platform_scale, platform_material), platform_pose)

    def create_platform(self, *platform_args):
        pass

    def save_as_stl(self):
        pass

    def set_visited(self, is_visited: bool):
        self._has_been_visited = is_visited

    def is_top_scaled(self):
        return self._platform_type.is_top_scaled()

    def is_bot_scaled(self):
        return self._platform_type.is_bot_scaled()

    def is_woody(self):
        return self._platform_type.is_woody()

    def is_stone(self):
        return self._platform_type.is_stone()

    def is_bland(self):
        return self._platform_type.is_bland()

    def is_brawn(self):
        return self._platform_type.is_brawn()

    def is_ground(self):
        return self._platform_type.get_shape() == PlatformShape.GROUND

    def is_root(self):
        return self._platform_type.get_shape() == PlatformShape.ROOT

    def is_goal(self):
        return self._platform_type.get_shape() == PlatformShape.GOAL

    def has_been_visited(self):
        return self._has_been_visited

    @staticmethod
    def get_platform_bevel_width():
        return PlatformConfig.PLATFORM_BEVEL_WIDTH

    @staticmethod
    def get_platform_height():
        return PlatformConfig.PLATFORM_HEIGHT

    @staticmethod
    def get_platform_top_surface_xy(platform_type):
        scale_const = 1.0
        if platform_type.is_top_scaled():
            scale_const = PlatformConfig.PLATFORM_SCALE_TOP_FACTOR

        platform_shape = platform_type.get_shape()

        if platform_shape == PlatformShape.BASE:
            return PlatformConfig.PLATFORM_SIZE_BASE * scale_const, PlatformConfig.PLATFORM_SIZE_BASE * scale_const

        if platform_shape == PlatformShape.WIDE:
            return PlatformConfig.PLATFORM_SIZE_SLIM * scale_const, PlatformConfig.PLATFORM_SIZE_BASE * scale_const

        if platform_shape == PlatformShape.LONG:
            return PlatformConfig.PLATFORM_SIZE_BASE * scale_const, PlatformConfig.PLATFORM_SIZE_SLIM * scale_const

        if platform_shape == PlatformShape.WALK:
            return PlatformConfig.PLATFORM_SIZE_WALK, PlatformConfig.PLATFORM_SIZE_BASE

        return PlatformConfig.PLATFORM_SIZE_BASE, PlatformConfig.PLATFORM_SIZE_BULK

    @staticmethod
    def get_platform_bounding_box(platform_type):
        platform_shape = platform_type.get_shape()

        if platform_shape == PlatformShape.BASE:
            return PlatformConfig.PLATFORM_SIZE_BASE, PlatformConfig.PLATFORM_SIZE_BASE

        if platform_shape == PlatformShape.WIDE:
            return PlatformConfig.PLATFORM_SIZE_SLIM, PlatformConfig.PLATFORM_SIZE_BASE

        if platform_shape == PlatformShape.LONG:
            return PlatformConfig.PLATFORM_SIZE_BASE, PlatformConfig.PLATFORM_SIZE_SLIM

        if platform_shape == PlatformShape.WALK:
            return PlatformConfig.PLATFORM_SIZE_WALK, PlatformConfig.PLATFORM_SIZE_BASE

        return PlatformConfig.PLATFORM_SIZE_BASE, PlatformConfig.PLATFORM_SIZE_BULK

    def get_platform_name(self):
        platform_shape = self.get_platform_type().get_shape()
        platform_scale = self.get_platform_type().get_scale()
        platform_material = self.get_platform_type().get_material()
        return "_".join([platform_shape, platform_scale, platform_material, self._platform_id])

    def get_platform_type(self):
        return self._platform_type

    def get_platform_pose(self):
        return self._platform_pose

    def get_mass(self):
        if self.get_platform_type().get_material() == PlatformMaterial.WOODY:
            return PlatformConfig.PLATFORM_MASS_WOODY
        return PlatformConfig.PLATFORM_MASS_STONE

    def get_platform_mesh_name(self):
        return self.get_platform_type().get_shape() + "_" + self.get_platform_type().get_scale()

    def get_platform_mesh_filename(self):
        return self.get_platform_mesh_name() + PlatformConfig.PLATFORM_MESH_TYPE
