class PlatformType:
    NORM = "norm"
    WIDE = "wide"
    LONG = "long"

    NOT_SCALED = "non"
    TOP_SCALED = "top"
    BOT_SCALED = "bot"

    @staticmethod
    def get_platform_type(shape_type, scale_type):
        return shape_type + "_" + str(scale_type)

    @staticmethod
    def get_shape_type(platform_type):
        return platform_type.split("_")[0]

    @staticmethod
    def get_scale_type(platform_type):
        return int(platform_type.split("_")[1])


class Platform:
    PLATFORM_MASS = 20.01

    PLATFORM_HEIGHT = 3.5
    PLATFORM_SIZE_NORM = 3.0
    PLATFORM_SIZE_LITE = 2.5
    PLATFORM_SIZE_LONG = 8.0

    PLATFORM_SCALE_FACTOR = .6
    PLATFORM_BEVEL_WIDTH = 0.75
    PLATFORM_BEVEL_SEGMENTS = 20

    PLATFORM_ROOT = "root_non"
    PLATFORM_GOAL = "goal_non"

    PLATFORM_MESH_TYPE = ".stl"

    def __init__(self, platform_name, platform_type, platform_pose, platform_color=None):
        self._platform_name = platform_name
        self._platform_type = platform_type
        self._platform_pose = platform_pose
        self._platform_color = platform_color

    def create_platform(self, *platform_args):
        pass

    def is_top_scaled(self):
        return self._platform_type.endswith(PlatformType.TOP_SCALED)

    def is_bot_scaled(self):
        return self._platform_type.endswith(PlatformType.BOT_SCALED)

    @staticmethod
    def get_platform_top_surface_xy(platform_type):
        shape_type = PlatformType.get_shape_type(platform_type)
        scale_type = PlatformType.get_scale_type(platform_type)

        scale_factor = Platform.PLATFORM_SCALE_FACTOR

        if shape_type == PlatformType.NORM:
            if scale_type == PlatformType.NOT_SCALED:
                return Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_NORM
            if scale_type == PlatformType.TOP_SCALED:
                return Platform.PLATFORM_SIZE_NORM * scale_factor, Platform.PLATFORM_SIZE_NORM * scale_factor
            if scale_type == PlatformType.BOT_SCALED:
                return Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_NORM

        if shape_type == PlatformType.WIDE:
            if scale_type == PlatformType.NOT_SCALED:
                return Platform.PLATFORM_SIZE_LITE, Platform.PLATFORM_SIZE_LONG
            if scale_type == PlatformType.TOP_SCALED:
                return Platform.PLATFORM_SIZE_LITE * scale_factor, Platform.PLATFORM_SIZE_LONG * scale_factor
            if scale_type == PlatformType.BOT_SCALED:
                return Platform.PLATFORM_SIZE_LITE, Platform.PLATFORM_SIZE_LONG

        if shape_type == PlatformType.LONG:
            if scale_type == PlatformType.NOT_SCALED:
                return Platform.PLATFORM_SIZE_LONG, Platform.PLATFORM_SIZE_LITE
            if scale_type == PlatformType.TOP_SCALED:
                return Platform.PLATFORM_SIZE_LONG * scale_factor, Platform.PLATFORM_SIZE_LITE * scale_factor
            if scale_type == PlatformType.BOT_SCALED:
                return Platform.PLATFORM_SIZE_LONG, Platform.PLATFORM_SIZE_LITE

        return Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_LONG

    @staticmethod
    def get_platform_bounding_box_dim(platform_type):
        shape_type = PlatformType.get_shape_type(platform_type)

        if shape_type == PlatformType.NORM:
            return Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_NORM

        if shape_type == PlatformType.WIDE:
            return Platform.PLATFORM_SIZE_LITE, Platform.PLATFORM_SIZE_LONG

        if shape_type == PlatformType.LONG:
            return Platform.PLATFORM_SIZE_LONG, Platform.PLATFORM_SIZE_LITE

        return Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_LONG

    def get_platform_name(self):
        return self._platform_name

    def get_platform_type(self):
        return self._platform_type

    def get_platform_pose(self):
        return self._platform_pose

    def get_platform_color(self):
        return self._platform_color

    def get_mass(self):
        return self.PLATFORM_MASS

    def get_platform_filename(self):
        return self.get_platform_type() + Platform.PLATFORM_MESH_TYPE
