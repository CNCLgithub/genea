class ExpType:
    GOAL = "goal"
    ACTION = "action"

    DIFFICULTY = "diff"
    HAND = "hand"
    MODEL = "model"

    @staticmethod
    def get_exp_type(input_string):
        if input_string == ExpType.GOAL:
            return ExpType.GOAL
        if input_string == ExpType.ACTION:
            return ExpType.ACTION
        if input_string == ExpType.DIFFICULTY:
            return ExpType.DIFFICULTY
        if input_string == ExpType.HAND:
            return ExpType.HAND
        return ExpType.MODEL
