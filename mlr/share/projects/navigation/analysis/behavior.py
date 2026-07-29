from mlr.share.projects.navigation.utils.analysis_utils import Trial, TrialKeys, Experiment, Participant, Quiz, \
    Questionnaire
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils


class ExperimentType:
    NAV_DIFFICULTY = "NAV_DIFFICULTY"


class NavDiffTrial(Trial):
    def __init__(self):
        super().__init__()
        self._trial_data_keys = [TrialKeys.TRIAL_NAME,
                                 TrialKeys.TRIAL_NUM,
                                 TrialKeys.TRIAL_VIDEO_FILENAME,
                                 TrialKeys.TRIAL_SLIDER_VALUE,
                                 TrialKeys.TRIAL_REACTION_TIME]

        self._trial_data_by_key = {key: None for key in self._trial_data_keys}

    # noinspection PyTypeChecker
    def add_trial_data(self, trial_data):
        self._trial_data_by_key[TrialKeys.TRIAL_NAME] = trial_data[TrialKeys.TRIAL_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_NUM] = int(trial_data[TrialKeys.TRIAL_NUM])
        self._trial_data_by_key[TrialKeys.TRIAL_SLIDER_VALUE] = int(trial_data[TrialKeys.TRIAL_SLIDER_VALUE])
        self._trial_data_by_key[TrialKeys.TRIAL_REACTION_TIME] = int(trial_data[TrialKeys.TRIAL_REACTION_TIME]) / 1000
        self._trial_data_by_key[TrialKeys.TRIAL_VIDEO_FILENAME] = trial_data[TrialKeys.TRIAL_VIDEO_FILENAME]


class NavDiffExperiment(Experiment):
    def __init__(self):
        super(NavDiffExperiment, self).__init__(30)

    @staticmethod
    def get_db_path():
        return PathUtils.join(PathUtils.get_prolific_dirpath(), "exp_diff_main.db")

    @staticmethod
    def get_db_table_name():
        return "genea_nav"

    def add_participant_data(self, participant_data, *arguments):
        excluded_participant_ids_list = arguments[0]

        if len(participant_data) == 0:
            return

        participant_id = self._extract_prolific_id(participant_data)
        if participant_id is None:
            participant_id = participant_data[0][self.UNIQUE_ID]

        if participant_id in excluded_participant_ids_list:
            return

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
                trial = NavDiffTrial()
                trial.add_trial_data(trial_data)
                participant.add_trial(trial)

    def apply_exclusion_criteria(self, *arguments):
        participant_id_list = self.get_participant_id_list()

        for pid in participant_id_list:
            participant_responses_dict = self.get_all_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE, [pid])
            participant_responses_list = [responses[0] for trial_num, responses in participant_responses_dict.items()]

            min_response = min(participant_responses_list)
            max_response = max(participant_responses_list)
            if max_response - min_response <= 10:
                Msg.print_info(f"Removing participant: {pid}")
                self.remove_participant(pid)
