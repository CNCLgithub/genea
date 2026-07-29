from mlr.share.projects.navigation.utils.analysis_utils import Experiment, Trial, TrialKeys, Participant
from mlr.share.projects.navigation.utils.file_utils import FileUtils


class ModelType:
    NAV_GENEA = "NAV_GENEA"
    NAV_TAMP = "NAV_TAMP"
    STABILITY = "STABILITY"
    VLM = "VLM"


class ModelData(Experiment):
    TRIAL_SUCCESS = "TRIAL_SUCCESS"
    COST_KE = "COST_KE"
    COST_CROCODDYL = "COST_CROCODDYL"
    COST_SYM_LEN = "COST_SYM_LEN"

    STIMULUS_NAME = 0
    STIMULUS_PLATFORM_COUNT = 1
    MOVE_NUM = 1
    VARIATION_NUM = 2
    RUN_NUM = 3
    PATH_AS_STR = 5
    PATH_SYM_LEN = 6
    PATH_ATTEMPTS = 7
    PATH_COST_KE = 8
    PATH_COST_CROCODDYL = 9

    def __init__(self, model_type):
        super(ModelData, self).__init__(None)
        self._model_type = model_type

    def add_participant_data(self, participant_id, *arguments):
        trial_name, trial_value = arguments

        if participant_id not in self._participants_by_participant_id_dict:
            self._participants_by_participant_id_dict[participant_id] = Participant(participant_id)

        participant = self._participants_by_participant_id_dict[participant_id]
        trial = Trial()
        trial.add_trial_data({TrialKeys.TRIAL_NAME: trial_name,
                              TrialKeys.TRIAL_SLIDER_VALUE: trial_value})
        participant.add_trial(trial)

    @staticmethod
    def apply_linking_function(trial_one, trial_two):
        return trial_one / (trial_one + trial_two)

    def get_model_type(self):
        return self._model_type

    def get_model_response_list(self, model_measure):
        return self.get_mean_trial_responses_dict(TrialKeys.TRIAL_Z_SCORE_VALUE, model_measure)

    def get_all_model_responses_list(self, model_measure):
        return self.get_all_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE, model_measure)

    @staticmethod
    def get_stability_scores(out_filepath):
        model_response_dict = {}

        for data in FileUtils.read_csv_file(out_filepath)[1:]:
            stim_num = int(data[0].split("_")[1])
            stability_score_list = [int(value) for value in data[2:]]
            model_response_dict[stim_num] = sum(stability_score_list)

        return model_response_dict

    @staticmethod
    def get_vlm_scores(out_filepath):
        model_response_dict = {}

        out_file_data = FileUtils.read_csv_file(out_filepath)
        for data in out_file_data:
            trial_str = data[0]
            trial_num = int(trial_str.split("_")[1])
            trial_var = int(trial_str.split("_")[2])
            if trial_var == 3:
                continue

            if trial_num not in model_response_dict:
                model_response_dict[trial_num] = []
            model_response_dict[trial_num].append(float(data[1]))

        out_response_dict = {}
        for trial_num, trial_list in model_response_dict.items():
            out_response_dict[trial_num] = sum(trial_list)
            out_response_dict[trial_num] /= len(trial_list)
        return out_response_dict
