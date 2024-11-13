
class ConfigUtils:
    DEFAULT_MAX_STEPS_PER_PLAN = 10

    DIFF_MAX_STEPS_PER_PLAN = 50

    IS_PHYSICS_ON = True
    DO_GRAB_OPTIMIZATION = True

    ONE_HAND_PENALTY = 30.0

    # options
    DEBUG_CPP = False
    RUN_PLANNER_OFFLINE = False

    NOISE_GAUSSIAN_STD = 0.01
    NOISE_THRESHOLD = 0.018  # if changed, remember to change this in run_physx.cpp as well


class NameUtils:
    NAME_RED = "red"
    NAME_YELLOW = "yellow"
    NAME_BLUE = "blue"
    NAME_MAUVE = "mauve"
    NAME_EMERALD = "emerald"
    NAME_BLACK = "black"

    NAME_DIFF_B = "b"
    NAME_DIFF_C = "c"

    BLOCK_NAMES_FIVE = [NAME_RED,
                        NAME_YELLOW,
                        NAME_BLUE,
                        NAME_MAUVE,
                        NAME_EMERALD]

    BLOCK_NAMES_THREE = [NAME_YELLOW,
                         NAME_MAUVE,
                         NAME_BLACK]

    BLOCK_NAMES_FOUR = [NAME_RED,
                        NAME_YELLOW,
                        NAME_MAUVE,
                        NAME_BLACK]

    TABLE = "table"
    TABLE_PLACE_LEFT = "TABLE_PLACE_LEFT"
    TABLE_PLACE_RIGHT = "TABLE_PLACE_RIGHT"

    NO_BLOCK_ON_TOP = "NO_BLOCK_ON_TOP"
    NO_BLOCK_UNDERNEATH = "NO_BLOCK_UNDERNEATH"

    ROBOT_HAND_LEFT = "ROBOT_HAND_LEFT"
    ROBOT_HAND_RIGHT = "ROBOT_HAND_RIGHT"
