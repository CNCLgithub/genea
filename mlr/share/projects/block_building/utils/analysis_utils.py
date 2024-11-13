import numpy as np

from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg


class TrialKeys:
    TRIAL_NAME = 'TrialName'
    TRIAL_NAME_DETAILED = 'TrialNameDetailed'
    TRIAL_NUM = 'TrialNumber'
    TRIAL_SLIDER_VALUE = 'SliderValue'
    TRIAL_REACTION_TIME = 'ReactionTime'
    TRIAL_INIT_IMAGE_NAME = 'TrialInitImageName'
    TRIAL_FINAL_IMAGE_NAME = 'TrialFinalImageName'
    TRIAL_LEFT_IMAGE_NAME = 'TrialLeftImageName'
    TRIAL_RIGHT_IMAGE_NAME = 'TrialRightImageName'
    TRIAL_LEFT_DETAIL = 'TrialLeftDetail'
    TRIAL_RIGHT_DETAIL = 'TrialRightDetail'

    TRIAL_VALUE = 'TrialValue'
    TRIAL_Z_SCORE_VALUE = 'TrialZScoreValue'


class Trial:
    def __init__(self):
        self._trial_data_keys = [TrialKeys.TRIAL_NAME, TrialKeys.TRIAL_VALUE]
        self._trial_data_by_key = {}

    def add_trial_data(self, trial_data):
        self._trial_data_by_key[TrialKeys.TRIAL_NAME] = trial_data[TrialKeys.TRIAL_NAME]
        self._trial_data_by_key[TrialKeys.TRIAL_VALUE] = trial_data[TrialKeys.TRIAL_VALUE]

    def set_response_value(self, value):
        self._trial_data_by_key[TrialKeys.TRIAL_VALUE] = value

    def set_z_scored_response_value(self, value):
        self._trial_data_by_key[TrialKeys.TRIAL_Z_SCORE_VALUE] = value

    def get_trial_name(self):
        return self._trial_data_by_key[TrialKeys.TRIAL_NAME]

    def get_detailed_trial_name(self):
        return self._trial_data_by_key[TrialKeys.TRIAL_NAME_DETAILED]

    def get_trial_data_keys(self):
        return self._trial_data_keys

    def get_trial_data_by_key(self, trial_key):
        return self._trial_data_by_key[trial_key]


class Quiz:
    QUIZ_QUESTION = 'Question'
    QUIZ_ANSWER = 'Answer'

    def __init__(self, question, answer):
        self.question = question
        self.answer = answer


class Questionnaire:
    QUESTIONNAIRE_QUESTION = 'Question'
    QUESTIONNAIRE_ANSWER = 'Answer'

    def __init__(self, question, answer):
        self.question = question
        self.answer = answer


class Participant:
    def __init__(self, participant_id):
        self._participant_id = participant_id

        self._trial_list_by_trial_name_dict = {}  # each trial name can have multiple "trial" objects
        self._quiz_list = []
        self._questionnaire_list = []

    def add_trial(self, trial):
        trial_name = trial.get_trial_name()

        if trial_name not in self._trial_list_by_trial_name_dict:
            self._trial_list_by_trial_name_dict[trial_name] = []

        self._trial_list_by_trial_name_dict[trial.get_trial_name()].append(trial)

    def add_quiz(self, quiz):
        self._quiz_list.append(quiz)

    def add_questionnaire(self, questionnaire):
        self._questionnaire_list.append(questionnaire)

    def get_participant_id(self):
        return self._participant_id

    def get_all_trial_names(self):
        return self._trial_list_by_trial_name_dict.keys()

    def get_all_trial_responses(self, trial_key):
        response_keys = []
        response_values = []
        for trial_name, trials_list in self._trial_list_by_trial_name_dict.items():
            for trial in trials_list:
                response_keys.append(trial_name)
                response_values.append(trial.get_trial_data_by_key(trial_key))

        return response_keys, response_values

    def get_all_trial_responses_by_trial_name(self, trial_key, trial_name):
        response_values = []
        if trial_name not in self._trial_list_by_trial_name_dict:
            Msg.print_info(f"Participant {self._participant_id}: trial {trial_name} not found")
            return response_values
        trials_list = self._trial_list_by_trial_name_dict[trial_name]
        for trial in trials_list:
            response_values.append(trial.get_trial_data_by_key(trial_key))
        return response_values

    def get_all_trial_responses_by_detailed_trial_name(self, trial_key, trial_name):
        response_values = []
        trials_list = self._trial_list_by_trial_name_dict[trial_name]
        for trial in trials_list:
            detailed_name = trial.get_trial_data_by_key(TrialKeys.TRIAL_NAME_DETAILED)
            response_values.append((detailed_name, trial.get_trial_data_by_key(trial_key)))
        return response_values

    def z_score_responses(self, trial_key):
        trial_responses_list = []

        for trial_name in self.get_all_trial_names():
            trials_list = self._trial_list_by_trial_name_dict[trial_name]
            for trial in trials_list:
                trial_responses_list.append(trial.get_trial_data_by_key(trial_key))

        z_scored_responses = ComputeUtils.zscore_list(trial_responses_list)

        index_visited = 0
        for trial_name in self.get_all_trial_names():
            trials_list = self._trial_list_by_trial_name_dict[trial_name]
            for trial in trials_list:
                trial.set_z_scored_response_value(z_scored_responses[index_visited])
                index_visited += 1

    @staticmethod
    def compute_replan_probability(trial_plan):
        trial_replan_cost_per_move = {m: 0 for m in range(1, 99)}
        previous_move = 999
        for move_index, curr_move in enumerate(map(int, trial_plan.split("_")[::2])):
            if move_index == 0:
                trial_replan_cost_per_move[0] = 0
                continue

            trial_replan_cost_per_move[curr_move] += 1

            if curr_move > previous_move:
                for m in range(1, previous_move + 1):
                    trial_replan_cost_per_move[m] += 1

            previous_move = curr_move

        for keys in trial_replan_cost_per_move.keys():
            if trial_replan_cost_per_move[keys] == -1:
                trial_replan_cost_per_move[keys] = 0

        return sum(trial_replan_cost_per_move.values())

    def set_replan_probability(self, trial_key):
        for trial_name in self.get_all_trial_names():
            trials_list = self._trial_list_by_trial_name_dict[trial_name]
            for trial in trials_list:
                trial_plan = trial.get_trial_data_by_key(trial_key)
                trial_replan_probability = self.compute_replan_probability(trial_plan)
                if trial_replan_probability > 30:
                    trial_replan_probability = 30
                trial.set_response_value(trial_replan_probability)


class Experiment:
    UNIQUE_ID = 'uniqueid'
    TRIAL_DATA = 'trialdata'
    PROLIFIC_ID = 'prolific_id'

    PHASE = 'Phase'
    PHASE_QUIZ = 'quiz'
    PHASE_QUESTIONNAIRE = 'questionnaire'
    PHASE_TRIAL = 'trial'

    def __init__(self, max_trial_length):
        self._max_trial_length = max_trial_length

        self._participants_by_participant_id_dict = {}

    @staticmethod
    def get_db_path():
        pass

    @staticmethod
    def get_db_table_name():
        pass

    @staticmethod
    def add_participant_data(participant_data, *arguments):
        pass

    def apply_exclusion_criteria(self, *arguments):
        pass

    def _extract_prolific_id(self, data):
        prolific_id = None
        for trial in data:
            trial_data = trial[self.TRIAL_DATA]
            if self.PROLIFIC_ID in trial_data:
                prolific_id = trial_data[self.PROLIFIC_ID]

        return prolific_id

    def compute_and_set_replan_probability(self, participant_id, trial_key):
        self._participants_by_participant_id_dict[participant_id].set_replan_probability(trial_key)

    def z_score_participant_responses(self, trial_key):
        for participant_id in self.get_participant_id_list():
            self._participants_by_participant_id_dict[participant_id].z_score_responses(trial_key)

    def remove_participant(self, participant_id):
        if participant_id in self._participants_by_participant_id_dict:
            del self._participants_by_participant_id_dict[participant_id]
            return
        Msg.print_error("Participant ID not found: " + participant_id)
        assert False

    def get_participant_id_list(self):
        return list(self._participants_by_participant_id_dict.keys())

    def get_participant_by_participant_id(self, participant_id):
        return self._participants_by_participant_id_dict[participant_id]

    def get_all_trial_names(self):
        participant_id = self.get_participant_id_list()[0]
        return sorted(self._participants_by_participant_id_dict[participant_id].get_all_trial_names())

    def get_trial_responses(self, participant_id, trial_key, trial_name):
        participant = self.get_participant_by_participant_id(participant_id)
        return participant.get_all_trial_responses_by_trial_name(trial_key, trial_name)

    def get_neg_exp_trial_responses_dict(self, trial_key, participant_id_list=None):
        trial_names_list = self.get_all_trial_names()

        if participant_id_list is None:
            participant_id_list = self.get_participant_id_list()
        if isinstance(participant_id_list, str):
            participant_id_list = [participant_id_list]

        response_values_dict = {key: [] for key in trial_names_list}

        for participant_id in participant_id_list:
            participant = self.get_participant_by_participant_id(participant_id)
            for trial_name in trial_names_list:
                trial_responses = participant.get_all_trial_responses_by_trial_name(trial_key, trial_name)
                trial_responses = [np.exp(-resp * 0.04) if resp > 0.0 else np.exp(-20) for resp in trial_responses]
                response_values_dict[trial_name].append(np.sum(trial_responses))

        for key in response_values_dict:
            response_values_dict[key] = np.mean(response_values_dict[key])

        return response_values_dict

    def get_min_trial_responses_dict(self, trial_key, participant_id_list=None):
        trial_names_list = self.get_all_trial_names()

        if participant_id_list is None:
            participant_id_list = self.get_participant_id_list()
        if isinstance(participant_id_list, str):
            participant_id_list = [participant_id_list]

        response_values_dict = {key: [] for key in trial_names_list}

        for participant_id in participant_id_list:
            participant = self.get_participant_by_participant_id(participant_id)
            for trial_name in trial_names_list:
                trial_responses = participant.get_all_trial_responses_by_trial_name(trial_key, trial_name)
                response_values_dict[trial_name].append(np.min(trial_responses))

        for key in response_values_dict:
            response_values_dict[key] = np.mean(response_values_dict[key])

        return response_values_dict

    def get_mean_trial_responses_dict(self, trial_key, participant_id_list=None):
        trial_names_list = self.get_all_trial_names()

        if participant_id_list is None:
            participant_id_list = self.get_participant_id_list()
        if isinstance(participant_id_list, str):
            participant_id_list = [participant_id_list]
        if isinstance(participant_id_list, int):
            participant_id_list = [participant_id_list]

        response_values_dict = {key: [] for key in trial_names_list}

        for participant_id in participant_id_list:
            participant = self.get_participant_by_participant_id(participant_id)
            for trial_name in trial_names_list:
                trial_responses = participant.get_all_trial_responses_by_trial_name(trial_key, trial_name)
                if len(trial_responses) == 0:
                    continue
                response_values_dict[trial_name].append(np.mean(trial_responses))

        for key in response_values_dict:
            responses = [resp for resp in response_values_dict[key] if not np.isnan(resp)]
            response_values_dict[key] = np.mean(responses)

        return response_values_dict

    def get_all_trial_responses_dict(self, trial_key, participant_id_list=None):
        trial_names_list = self.get_all_trial_names()

        if participant_id_list is None:
            participant_id_list = self.get_participant_id_list()
        if isinstance(participant_id_list, str):
            participant_id_list = [participant_id_list]
        if isinstance(participant_id_list, int):
            participant_id_list = [participant_id_list]

        response_values_dict = {key: [] for key in trial_names_list}

        for participant_id in participant_id_list:
            participant = self.get_participant_by_participant_id(participant_id)
            for trial_name in trial_names_list:
                trial_responses = participant.get_all_trial_responses_by_trial_name(trial_key, trial_name)
                response_values_dict[trial_name].append(np.mean(trial_responses))

        return response_values_dict

    def get_all_trial_responses_by_detailed_name_dict(self, trial_key, participant_id_list=None):
        trial_names_list = self.get_all_trial_names()

        if participant_id_list is None:
            participant_id_list = self.get_participant_id_list()
        if isinstance(participant_id_list, str):
            participant_id_list = [participant_id_list]
        if isinstance(participant_id_list, int):
            participant_id_list = [participant_id_list]

        response_values_dict = {key: [] for key in trial_names_list}

        for participant_id in participant_id_list:
            participant = self.get_participant_by_participant_id(participant_id)
            for trial_name in trial_names_list:
                trial_responses = participant.get_all_trial_responses_by_detailed_trial_name(trial_key, trial_name)
                response_values_dict[trial_name].append(trial_responses)

        return response_values_dict
