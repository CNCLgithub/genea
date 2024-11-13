from mlr.share.projects.block_building.utils.analysis_utils import Trial, TrialKeys, Experiment, Participant, Quiz, \
    Questionnaire
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class ExperimentType:
    HANDS = "HANDS"
    ACTION_GOAL = "ACTION_GOAL"

    DIFFICULTY = "DIFFICULTY_"
    DIFFICULTY_DIFF = "DIFFICULTY_DIFF"
    DIFFICULTY_TIME = "DIFFICULTY_TIME"
    DIFFICULTY_BUILD = "DIFFICULTY_BUILD"


class ActionGoalTrial(Trial):
    TRIALS_TO_FLIP = ["action_b", "action_d", "action_i", "goal_a", "goal_c", "goal_d"]

    def __init__(self, trial_name, trial_name_detailed):
        super().__init__()
        self._trial_data_keys = [TrialKeys.TRIAL_NAME,
                                 TrialKeys.TRIAL_NUM,
                                 TrialKeys.TRIAL_SLIDER_VALUE,
                                 TrialKeys.TRIAL_REACTION_TIME,
                                 TrialKeys.TRIAL_INIT_IMAGE_NAME,
                                 TrialKeys.TRIAL_FINAL_IMAGE_NAME,
                                 TrialKeys.TRIAL_LEFT_IMAGE_NAME,
                                 TrialKeys.TRIAL_RIGHT_IMAGE_NAME,
                                 TrialKeys.TRIAL_LEFT_DETAIL,
                                 TrialKeys.TRIAL_RIGHT_DETAIL]

        self._trial_data_by_key = {key: None for key in self._trial_data_keys}

        self._trial_name = trial_name
        self._trial_name_detailed = trial_name_detailed

    # noinspection PyTypeChecker
    def correct_slider_values(self):  # refer to the stimuli set to see why this is necessary
        def _flip(value):
            return 50 + (50 - int(value))

        if self._trial_name in self.TRIALS_TO_FLIP:
            if "1" in self._trial_name_detailed:
                slider_value = self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE]
                self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE] = _flip(slider_value)
            elif "3" in self._trial_name_detailed:
                slider_value = self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE]
                self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE] = _flip(slider_value)
            return
        if "2" in self._trial_name_detailed:
            slider_value = self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE]
            self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE] = _flip(slider_value)

    # noinspection PyTypeChecker
    def add_trial_data(self, trial_data):
        self._trial_data_by_key[TrialKeys.TRIAL_NAME] = self._trial_name
        self._trial_data_by_key[TrialKeys.TRIAL_NAME_DETAILED] = self._trial_name_detailed
        self._trial_data_by_key[TrialKeys.TRIAL_NUM] = int(trial_data[TrialKeys.TRIAL_NUM])
        self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE] = int(trial_data[TrialKeys.TRIAL_SLIDER_VALUE])
        self._trial_data_by_key[TrialKeys.TRIAL_REACTION_TIME] = int(trial_data[TrialKeys.TRIAL_REACTION_TIME])
        self._trial_data_by_key[TrialKeys.TRIAL_INIT_IMAGE_NAME] = trial_data[TrialKeys.TRIAL_INIT_IMAGE_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_FINAL_IMAGE_NAME] = trial_data[TrialKeys.TRIAL_FINAL_IMAGE_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_LEFT_IMAGE_NAME] = trial_data[TrialKeys.TRIAL_LEFT_IMAGE_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_RIGHT_IMAGE_NAME] = trial_data[TrialKeys.TRIAL_RIGHT_IMAGE_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_LEFT_DETAIL] = trial_data[TrialKeys.TRIAL_LEFT_DETAIL]
        self._trial_data_by_key[TrialKeys.TRIAL_RIGHT_DETAIL] = trial_data[TrialKeys.TRIAL_RIGHT_DETAIL]


class HandsTrial(Trial):
    def __init__(self):
        super().__init__()
        self._trial_data_keys = [TrialKeys.TRIAL_NAME,
                                 TrialKeys.TRIAL_NUM,
                                 TrialKeys.TRIAL_SLIDER_VALUE,
                                 TrialKeys.TRIAL_REACTION_TIME,
                                 TrialKeys.TRIAL_INIT_IMAGE_NAME,
                                 TrialKeys.TRIAL_FINAL_IMAGE_NAME]

        self._trial_data_by_key = {key: None for key in self._trial_data_keys}

    # noinspection PyTypeChecker
    def add_trial_data(self, trial_data):
        self._trial_data_by_key[TrialKeys.TRIAL_NAME] = trial_data[TrialKeys.TRIAL_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_NUM] = int(trial_data[TrialKeys.TRIAL_NUM])
        self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE] = int(trial_data[TrialKeys.TRIAL_SLIDER_VALUE])
        self._trial_data_by_key[TrialKeys.TRIAL_REACTION_TIME] = int(trial_data[TrialKeys.TRIAL_REACTION_TIME])
        self._trial_data_by_key[TrialKeys.TRIAL_INIT_IMAGE_NAME] = trial_data[TrialKeys.TRIAL_INIT_IMAGE_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_FINAL_IMAGE_NAME] = trial_data[TrialKeys.TRIAL_FINAL_IMAGE_NAME]


class DifficultyExperiment(Experiment):
    DIFF_TRIAL_NAMES = ['7_1', '7_2', '8_1', '8_2', '9_1', '9_2',
                        '11_1', '11_2', '12_1', '12_2', '13_1', '13_2', '14_1', '14_2', '15_1', '15_2',
                        '16_1', '16_2', '17_1', '17_2', '18_1', '18_2', '19_1', '19_2', '20_1', '20_2',
                        '21_1', '21_2', '22_1', '22_2', '23_1', '23_2', '25_1', '25_2',
                        '26_1', '26_2', '27_1', '27_2', '28_1', '28_2', '29_1', '29_2', '30_1', '30_2',
                        '31_1', '31_2', '33_1', '33_2', '34_1', '34_2']

    def __init__(self):
        super(DifficultyExperiment, self).__init__(58)

    def add_participant_data(self, participant_id, *arguments):
        trial_name, trial_value = arguments

        if participant_id not in self._participants_by_participant_id_dict:
            self._participants_by_participant_id_dict[participant_id] = Participant(participant_id)

        participant = self._participants_by_participant_id_dict[participant_id]
        trial = Trial()
        trial.add_trial_data({TrialKeys.TRIAL_NAME: trial_name,
                              TrialKeys.TRIAL_VALUE: trial_value})
        participant.add_trial(trial)


class ActionGoalExperiment(Experiment):
    def __init__(self):
        super(ActionGoalExperiment, self).__init__(44)

    @staticmethod
    def get_db_path():
        return PathUtils.join(PathUtils.get_out_human_data_dirpath(), "action_goal_participants.db")

    @staticmethod
    def get_db_table_name():
        return "blocks_social"

    def add_participant_data(self, participant_data):
        if len(participant_data) == 0:
            return

        participant_id = self._extract_prolific_id(participant_data)
        if participant_id is None:
            participant_id = participant_data[0][self.UNIQUE_ID]

        if participant_id not in self._participants_by_participant_id_dict:
            self._participants_by_participant_id_dict[participant_id] = Participant(participant_id)

        participant = self._participants_by_participant_id_dict[participant_id]

        for trial in participant_data:
            trial_data = trial[self.TRIAL_DATA]

            if Experiment.PROLIFIC_ID in trial_data:
                continue

            if trial["current_trial"] > self._max_trial_length:  # if participants refreshed/submitted multiple HITs
                continue

            if trial_data[self.PHASE] == self.PHASE_QUIZ:
                question = trial_data[Quiz.QUIZ_QUESTION]
                answer = trial_data[Quiz.QUIZ_ANSWER]
                participant.add_quiz(Quiz(question, answer))

            elif trial_data[self.PHASE] == self.PHASE_QUESTIONNAIRE:
                question = trial_data[Questionnaire.QUESTIONNAIRE_QUESTION]
                answer = trial_data[Questionnaire.QUESTIONNAIRE_ANSWER]
                participant.add_questionnaire(Questionnaire(question, answer))

            elif trial_data[self.PHASE] == self.PHASE_TRIAL:
                trial_name = trial_data[TrialKeys.TRIAL_NAME]
                trial_name_detailed = trial_data[TrialKeys.TRIAL_INIT_IMAGE_NAME].split(".")[0]

                trial = ActionGoalTrial(trial_name, trial_name_detailed)
                trial.add_trial_data(trial_data)
                trial.correct_slider_values()
                participant.add_trial(trial)

    def apply_exclusion_criteria(self, *arguments):
        def _bin_responses(response_list):
            result = []
            for response in response_list:
                if response == 50:
                    result.append(50)
                elif response < 50:
                    result.append(0)
                elif response > 50:
                    result.append(100)
            return result

        def _check_attention(slider_res):
            return max(slider_res.count(0), slider_res.count(100))

        bad_participant_id_list = []

        participant_id_list = self.get_participant_id_list()
        for participant_id in participant_id_list:
            trial_key = TrialKeys.TRIAL_SLIDER_VALUE

            slider_responses_f = self.get_trial_responses(participant_id, trial_key, "action_f")
            slider_responses_g = self.get_trial_responses(participant_id, trial_key, "action_g")

            counter1 = _check_attention(_bin_responses(slider_responses_f))
            counter2 = _check_attention(_bin_responses(slider_responses_g))

            if counter1 < 3 and counter2 < 3:
                print("Subject " + str(participant_id) + " failed attentional check: " + str(6 - (counter1 + counter2)))
                bad_participant_id_list.append(participant_id)
                continue

        for participant_id in bad_participant_id_list:
            self.remove_participant(participant_id)

        print("Subjects excluded: " + str(len(bad_participant_id_list)))


class HandsExperiment(Experiment):
    def __init__(self):
        super(HandsExperiment, self).__init__(30)

    @staticmethod
    def get_db_path():
        return PathUtils.join(PathUtils.get_out_human_data_dirpath(), "hands_participants.db")

    @staticmethod
    def get_db_table_name():
        return "blocks_social"

    def add_participant_data(self, participant_data):
        if len(participant_data) == 0:
            return

        participant_id = self._extract_prolific_id(participant_data)
        if participant_id is None:
            participant_id = participant_data[0][self.UNIQUE_ID]

        if participant_id not in self._participants_by_participant_id_dict:
            self._participants_by_participant_id_dict[participant_id] = Participant(participant_id)

        participant = self._participants_by_participant_id_dict[participant_id]

        for trial in participant_data:
            trial_data = trial[self.TRIAL_DATA]

            if Experiment.PROLIFIC_ID in trial_data:
                continue

            if trial["current_trial"] > self._max_trial_length:  # if participants refreshed/submitted multiple HITs
                continue

            if trial_data[self.PHASE] == self.PHASE_QUIZ:
                question = trial_data[Quiz.QUIZ_QUESTION]
                answer = trial_data[Quiz.QUIZ_ANSWER]
                participant.add_quiz(Quiz(question, answer))

            elif trial_data[self.PHASE] == self.PHASE_QUESTIONNAIRE:
                question = trial_data[Questionnaire.QUESTIONNAIRE_QUESTION]
                answer = trial_data[Questionnaire.QUESTIONNAIRE_ANSWER]
                participant.add_questionnaire(Questionnaire(question, answer))

            elif trial_data[self.PHASE] == self.PHASE_TRIAL:
                trial = HandsTrial()
                trial.add_trial_data(trial_data)
                participant.add_trial(trial)

    def apply_exclusion_criteria(self, *arguments):
        def _bin_responses(response_list):
            result = []
            for response in response_list:
                if response <= 30:
                    result.append(0)
                else:
                    result.append(100)
            return result

        def _check_attention(slider_res):
            return slider_res.count(0)

        bad_participant_id_list = []

        participant_id_list = self.get_participant_id_list()
        for participant_id in participant_id_list:
            trial_key = TrialKeys.TRIAL_SLIDER_VALUE
            slider_responses_2 = self.get_trial_responses(participant_id, trial_key, "hand_2")
            slider_responses_5 = self.get_trial_responses(participant_id, trial_key, "hand_5")
            slider_responses_14 = self.get_trial_responses(participant_id, trial_key, "hand_14")

            counter1 = _check_attention(_bin_responses(slider_responses_2))
            counter2 = _check_attention(_bin_responses(slider_responses_5))
            counter3 = _check_attention(_bin_responses(slider_responses_14))

            if not counter1 + counter2 + counter3 == 3:
                print("Subject " + str(participant_id) + " failed attentional check: " + str(6 - (counter1 + counter2)))
                bad_participant_id_list.append(participant_id)
                continue

        for participant_id in bad_participant_id_list:
            self.remove_participant(participant_id)

        print("Subjects excluded: " + str(len(bad_participant_id_list)))
