
class _RobotHandState:
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"

    def __init__(self, ):
        self.state = self.AVAILABLE

    def is_available(self):
        return self.state == self.AVAILABLE

    def is_occupied(self):
        return self.state == self.OCCUPIED

    def update_state(self, new_state):
        self.state = new_state


class RobotHand:
    def __init__(self):
        self.current_state = _RobotHandState()
        self.currently_holding = None

    def get_contents(self):
        return self.currently_holding

    def is_available(self):
        return self.current_state.is_available()

    def is_occupied(self):
        return self.current_state.is_occupied()

    def grab_block(self, block):
        self.currently_holding = block
        self.current_state.update_state(_RobotHandState.OCCUPIED)

    def release_block(self):
        self.currently_holding = None
        self.current_state.update_state(_RobotHandState.AVAILABLE)


class RobotActions:
    IDLE_LEFT = "IDLE_LEFT"
    IDLE_RIGHT = "IDLE_RIGHT"

    GRAB = "GRAB"
    GRAB_LEFT_HAND = "GRAB_LEFT_HAND"
    GRAB_RIGHT_HAND = "GRAB_RIGHT_HAND"

    PLACE = "PLACE"
    PLACE_LEFT_HAND = "PLACE_LEFT_HAND"
    PLACE_RIGHT_HAND = "PLACE_RIGHT_HAND"

    FIX = "FIX"
    FIX_LEFT_HAND = "FIX_LEFT_HAND"
    FIX_RIGHT_HAND = "FIX_RIGHT_HAND"

    # let the block remain in the hand i.e., keep the block grabbed
    DO_NOTHING_G_LEFT_HAND = "DO_NOTHING_G_LEFT_HAND"
    DO_NOTHING_G_RIGHT_HAND = "DO_NOTHING_G_RIGHT_HAND"

    INACTIVE = [IDLE_LEFT, IDLE_RIGHT,
                DO_NOTHING_G_LEFT_HAND, DO_NOTHING_G_RIGHT_HAND]

    def __init__(self, action, block):
        self._action = action
        self._block = block

    @staticmethod
    def get_next_possible_actions(current_action):
        if current_action == RobotActions.GRAB_LEFT_HAND:
            return [RobotActions.FIX_LEFT_HAND, RobotActions.PLACE_LEFT_HAND, RobotActions.DO_NOTHING_G_LEFT_HAND]
        elif current_action == RobotActions.GRAB_RIGHT_HAND:
            return [RobotActions.FIX_RIGHT_HAND, RobotActions.PLACE_RIGHT_HAND, RobotActions.DO_NOTHING_G_RIGHT_HAND]
        elif current_action == RobotActions.PLACE_LEFT_HAND or current_action == RobotActions.FIX_LEFT_HAND:
            return [RobotActions.GRAB_LEFT_HAND, RobotActions.IDLE_LEFT]
        elif current_action == RobotActions.PLACE_RIGHT_HAND or current_action == RobotActions.FIX_RIGHT_HAND:
            return [RobotActions.GRAB_RIGHT_HAND, RobotActions.IDLE_RIGHT]
        elif current_action == RobotActions.DO_NOTHING_G_LEFT_HAND:
            return [RobotActions.FIX_LEFT_HAND, RobotActions.PLACE_LEFT_HAND, RobotActions.DO_NOTHING_G_LEFT_HAND]
        elif current_action == RobotActions.DO_NOTHING_G_RIGHT_HAND:
            return [RobotActions.FIX_RIGHT_HAND, RobotActions.PLACE_RIGHT_HAND, RobotActions.DO_NOTHING_G_RIGHT_HAND]
        elif current_action == RobotActions.IDLE_LEFT:
            return [RobotActions.GRAB_LEFT_HAND, RobotActions.IDLE_LEFT]
        elif current_action == RobotActions.IDLE_RIGHT:
            return [RobotActions.GRAB_RIGHT_HAND, RobotActions.IDLE_RIGHT]

    @staticmethod
    def get_next_possible_actions_diff(current_action):
        if current_action == RobotActions.GRAB_LEFT_HAND:
            return [RobotActions.FIX_LEFT_HAND]
        elif current_action == RobotActions.GRAB_RIGHT_HAND:
            return [RobotActions.FIX_RIGHT_HAND]
        elif current_action == RobotActions.FIX_LEFT_HAND or current_action == RobotActions.PLACE_LEFT_HAND:
            return [RobotActions.GRAB_LEFT_HAND, RobotActions.IDLE_LEFT]
        elif current_action == RobotActions.FIX_RIGHT_HAND or current_action == RobotActions.PLACE_RIGHT_HAND:
            return [RobotActions.GRAB_RIGHT_HAND, RobotActions.IDLE_RIGHT]
        elif current_action == RobotActions.IDLE_LEFT:
            return [RobotActions.GRAB_LEFT_HAND, RobotActions.IDLE_LEFT]
        elif current_action == RobotActions.IDLE_RIGHT:
            return [RobotActions.GRAB_RIGHT_HAND, RobotActions.IDLE_RIGHT]

    @staticmethod
    def is_inactive(action):
        if action in RobotActions.INACTIVE:
            return True
        return False

    @staticmethod
    def are_both_inactive(action1, action2):
        if action1 in RobotActions.INACTIVE and action2 in RobotActions.INACTIVE:
            return True
        return False

    @staticmethod
    def is_one_action_idle(action1, action2):
        inactive = [RobotActions.IDLE_LEFT, RobotActions.IDLE_RIGHT]

        if action1 in inactive or action2 in inactive:
            return True
        return False
