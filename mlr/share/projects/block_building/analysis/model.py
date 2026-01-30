import csv

import numpy as np

from scipy.stats import stats

from mlr.share.projects.block_building.analysis.behavior import ExperimentType
from mlr.share.projects.block_building.utils.analysis_utils import TrialKeys, Experiment, Participant, Trial
from mlr.share.projects.block_building.utils.core_utils import ConfigUtils
from mlr.share.projects.block_building.utils.file_utils import FileUtils
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class ModelType:
    PHYSICS_ON = "PHYSICS_ON"
    PHYSICS_ON_SSA = "PHYSICS_ON_SSA"
    PHYSICS_OFF = "PHYSICS_OFF"
    PHYSICS_OFF_SSA = "PHYSICS_OFF_SSA"
    HEURISTICS = "HEURISTICS"
    STABILITY = "STABILITY"

    VLM = "VLM"


class ModelData(Experiment):
    TRIAL_NAME = 0
    TRIAL_NUM_OF_HANDS = 1
    TRIAL_SYMBOLIC_PLAN = 2
    TRIAL_SYMBOLIC_PLAN_DETAILED = 3
    TRIAL_COST = 4
    TRIAL_ENERGY = 5

    ENERGY = "ENERGY"
    COST = "COST"

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

    def get_hands_response_list(self, model_measure, exp_temp):
        new_response_dict = {}

        trial_names_list = self.get_all_trial_names()
        old_response_dict = self.get_min_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE, model_measure)
        prob_response_dict = self.get_prob_trial_responses_dict(TrialKeys.TRIAL_Z_SCORE_VALUE, model_measure, exp_temp)

        for trials in trial_names_list:
            new_trial_name = trials[:-2]

            if new_trial_name in new_response_dict:
                continue

            trial_1 = old_response_dict[new_trial_name + "_1"]
            trial_2 = old_response_dict[new_trial_name + "_2"]

            if trial_1 == 0:
                new_response_dict[new_trial_name] = 1.0
                continue

            if trial_2 == 0:
                new_response_dict[new_trial_name] = 0.0
                continue

            trial_1 = prob_response_dict[new_trial_name + "_1"]
            trial_2 = prob_response_dict[new_trial_name + "_2"]

            new_response_dict[new_trial_name] = trial_2 / (trial_1 + trial_2)

        return new_response_dict

    def get_action_goal_response_list(self, model_measure, exp_temp):
        new_response_dict = {}

        trial_names_list = self.get_all_trial_names()
        old_response_dict = self.get_prob_trial_responses_dict(TrialKeys.TRIAL_Z_SCORE_VALUE, model_measure, exp_temp)

        for trials in trial_names_list:
            new_trial_name = trials[:-1]

            if new_trial_name in new_response_dict:
                continue

            trial_1 = old_response_dict[new_trial_name + "1"]
            trial_2 = old_response_dict[new_trial_name + "2"]

            numerator = 0.0
            if new_trial_name == "action_a":
                numerator = np.mean(trial_2)
            elif new_trial_name == "action_b":
                numerator = np.mean(trial_1)
            elif new_trial_name == "action_c":
                numerator = np.mean(trial_1)
            elif new_trial_name == "action_d":
                numerator = np.mean(trial_2)
            elif new_trial_name == "action_e":
                numerator = np.mean(trial_2)
            elif new_trial_name == "action_f":
                numerator = np.mean(trial_1)
            elif new_trial_name == "action_g":
                numerator = np.mean(trial_1)
            elif new_trial_name == "action_h":
                numerator = np.mean(trial_1)
            elif new_trial_name == "action_i":
                numerator = np.mean(trial_2)
            elif new_trial_name == "goal_a":
                numerator = np.mean(trial_1)
            elif new_trial_name == "goal_b":
                numerator = np.mean(trial_2)
            elif new_trial_name == "goal_c":
                numerator = np.mean(trial_1)
            elif new_trial_name == "goal_d":
                numerator = np.mean(trial_2)
            elif new_trial_name == "goal_e":
                numerator = np.mean(trial_1)
            elif new_trial_name == "goal_f":
                numerator = np.mean(trial_1)
            elif new_trial_name == "goal_g":
                numerator = np.mean(trial_2)
            elif new_trial_name == "goal_h":
                numerator = np.mean(trial_2)

            new_response_dict[new_trial_name] = numerator / (trial_1 + trial_2)

        return new_response_dict

    def get_model_response_list(self, experiment_type, model_measure):
        model_type = self.get_model_type()

        exp_temp = ConfigUtils.TEMP_DEFAULT
        if model_measure == ModelData.ENERGY:
            exp_temp = ConfigUtils.TEMP_ENERGY
        elif model_measure == ModelData.COST:
            exp_temp = ConfigUtils.TEMP_COST

        if model_type == ModelType.PHYSICS_ON_SSA or model_type == ModelType.PHYSICS_OFF_SSA:
            exp_temp = ConfigUtils.TEMP_SSA

        if experiment_type == ExperimentType.HANDS:
            model_response_dict = self.get_hands_response_list(model_measure, exp_temp)
        elif experiment_type == ExperimentType.ACTION_GOAL:
            model_response_dict = self.get_action_goal_response_list(model_measure, exp_temp)
        else:
            model_response_dict = self.get_mean_trial_responses_dict(TrialKeys.TRIAL_Z_SCORE_VALUE, model_measure)

        return model_response_dict

    @staticmethod
    def get_hand_heuristics_stability():
        model_response_dict = {}

        trial_names_list = ["hand_" + str(i) for i in range(1, 23)]

        for trial_name in trial_names_list:
            model_response_dict[trial_name] = .5 + np.random.random() * 1000

        return model_response_dict

    @staticmethod
    def get_action_goal_heuristics_stability():
        model_response_dict = {}

        trial_names_list = ["action_" + chr(i) for i in range(ord('a'), ord('i') + 1)]
        trial_names_list += ["goal_" + chr(i) for i in range(ord('a'), ord('h') + 1)]

        for trial_name in trial_names_list:
            model_response_dict[trial_name] = .5 + np.random.random() * 1000

        return model_response_dict

    @staticmethod
    def get_diff_heuristics_stability(noise_value=ConfigUtils.NOISE_GAUSSIAN_STD):
        return_dict = {}
        risk_by_trial_name = {}
        sub_dir_list = FileUtils.get_dir_list_in_directory(PathUtils.get_risk_files_dirpath(noise_value))
        for sub_dir in sub_dir_list:
            trial_name = FileUtils.get_file_basename(sub_dir)
            if "diff" in trial_name:
                trial_name = trial_name.split("diff_")[1]
            dir_files = FileUtils.get_files_in_directory(sub_dir)

            risk_by_trial_name[trial_name] = []

            for file_path in dir_files:
                if FileUtils.get_file_basename(file_path).endswith(PathUtils.DOT_TXT):
                    with open(file_path) as risk_file:
                        for line in risk_file:
                            risk_by_trial_name[trial_name].append(float(line))

            return_dict[trial_name] = sum(risk_by_trial_name[trial_name]) / len(risk_by_trial_name[trial_name])
        return return_dict

    @staticmethod
    def get_hand_heuristics():
        """
        total blocks that move going from initial to final configuration
        0   : if only one block moved
        100 : if more than 1 block moved
        :return:
        """
        model_response_dict = {}

        trial_names_list = ["hand_" + str(i) for i in range(1, 23)]

        scores = [100,  # hand_1
                  0,  # hand_2
                  100,  # hand_3
                  100,  # hand_4
                  0,  # hand_5
                  100,  # hand_6
                  100,  # hand_7
                  100,  # hand_8
                  100,  # hand_9
                  0,  # hand_10
                  100,  # hand_11
                  100,  # hand_12
                  100,
                  0,
                  100,
                  100,
                  100,
                  100,
                  100,
                  100,
                  100,
                  100]

        z_scored_scores = stats.zscore(scores)

        for trial_name in trial_names_list:
            model_response_dict[trial_name] = z_scored_scores[trial_names_list.index(trial_name)]

        return model_response_dict

    @staticmethod
    def get_action_goal_heuristics():
        """
        For action  : not considering the grabbed block(s),
                      select the option where lesser number of blocks remain to be moved

        For goal    : not considering the grabbed block(s),
                      select option with min total distance covered by blocks

        NOTE        : all values are based on the behavioral stimuli (flipped where necessary)

        :return:
        """

        model_response_dict = {}

        trial_names_list = ["action_" + chr(i) for i in range(ord('a'), ord('i') + 1)]
        trial_names_list += ["goal_" + chr(i) for i in range(ord('a'), ord('h') + 1)]

        scores = [0.33,  # action_a
                  1,  # action_b (flipped)
                  1,
                  1,  # action_d (flipped)
                  0,
                  0,
                  1,
                  1,
                  .5,  # action_i (flipped)
                  .5,  # goal_a (flipped)
                  0.90,   # goal_b
                  0.89,   # goal_c (flipped)
                  .5,  # goal_d (flipped)
                  .5,  # goal_e
                  .5,  # goal_f
                  .5,  # goal_g
                  .5]

        z_scored_scores = stats.zscore(scores)

        for trial_name in trial_names_list:
            model_response_dict[trial_name] = z_scored_scores[trial_names_list.index(trial_name)]

        return model_response_dict

    @staticmethod
    def get_diff_heuristics():
        """
        total number of blocks moved going from initial to final configuration
        """

        def get_diff_trial_names_list():
            exp_trial_names_list = []

            trials_info_list_filename = PathUtils.join(PathUtils.get_exp_data_dirpath(), "trials_list.csv")
            with open(trials_info_list_filename, "r") as trials_info_list_file:
                trials_info_list = csv.reader(trials_info_list_file, delimiter=',')
                trials_info_list = list(trials_info_list)
                for trial_info in trials_info_list[1:]:
                    exp_type = trial_info[0]
                    if not "diff" == exp_type:
                        continue
                    exp_trial_names_list.append(trial_info[1])
            return exp_trial_names_list

        def _diff_block_nums(exp_trial_name):
            total_b_blocks = 0
            total_c_blocks = 0
            if exp_trial_name == "7_1":
                total_b_blocks = 3
            elif exp_trial_name == "7_2":
                total_b_blocks = 10
            elif exp_trial_name == "8_1":
                total_b_blocks = 5
            elif exp_trial_name == "8_2":
                total_b_blocks = 15
            elif exp_trial_name == "9_1":
                total_b_blocks = 10
            elif exp_trial_name == "9_2":
                total_b_blocks = 10
            elif exp_trial_name == "11_1":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_name == "11_2":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_name == "12_1":
                total_b_blocks = 2
            elif exp_trial_name == "12_2":
                total_b_blocks = 10
            elif exp_trial_name == "13_1":
                total_b_blocks = 5
                total_c_blocks = 17
            elif exp_trial_name == "13_2":
                total_b_blocks = 17
                total_c_blocks = 5
            elif exp_trial_name == "14_1":
                total_b_blocks = 10
            elif exp_trial_name == "14_2":
                total_b_blocks = 10
            elif exp_trial_name == "15_1":
                total_b_blocks = 5
            elif exp_trial_name == "15_2":
                total_b_blocks = 13
            elif exp_trial_name == "16_1":
                total_b_blocks = 5
            elif exp_trial_name == "16_2":
                total_b_blocks = 15
            elif exp_trial_name == "17_1":
                total_b_blocks = 2
                total_c_blocks = 8
            elif exp_trial_name == "17_2":
                total_b_blocks = 8
                total_c_blocks = 2
            elif exp_trial_name == "18_1":
                total_b_blocks = 8
                total_c_blocks = 7
            elif exp_trial_name == "18_2":
                total_b_blocks = 8
                total_c_blocks = 7
            elif exp_trial_name == "19_1":
                total_b_blocks = 10
            elif exp_trial_name == "19_2":
                total_b_blocks = 5
                total_c_blocks = 5
            elif exp_trial_name == "20_1":
                total_b_blocks = 21
            elif exp_trial_name == "20_2":
                total_b_blocks = 21
            elif exp_trial_name == "21_1":
                total_b_blocks = 4
            elif exp_trial_name == "21_2":
                total_b_blocks = 14
            elif exp_trial_name == "22_1":
                total_b_blocks = 3
            elif exp_trial_name == "22_2":
                total_b_blocks = 15
            elif exp_trial_name == "23_1":
                total_b_blocks = 6
            elif exp_trial_name == "23_2":
                total_b_blocks = 10
            elif exp_trial_name == "25_1":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_name == "25_2":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_name == "26_1":
                total_b_blocks = 4
            elif exp_trial_name == "26_2":
                total_b_blocks = 16
            elif exp_trial_name == "27_1":
                total_b_blocks = 5
                total_c_blocks = 17
            elif exp_trial_name == "27_2":
                total_b_blocks = 17
                total_c_blocks = 5
            elif exp_trial_name == "28_1":
                total_b_blocks = 12
            elif exp_trial_name == "28_2":
                total_b_blocks = 12
            elif exp_trial_name == "29_1":
                total_b_blocks = 3
            elif exp_trial_name == "29_2":
                total_b_blocks = 15
            elif exp_trial_name == "30_1":
                total_b_blocks = 3
            elif exp_trial_name == "30_2":
                total_b_blocks = 15
            elif exp_trial_name == "31_1":
                total_b_blocks = 2
                total_c_blocks = 8
            elif exp_trial_name == "31_2":
                total_b_blocks = 10
            elif exp_trial_name == "33_1":
                total_b_blocks = 12
            elif exp_trial_name == "33_2":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_name == "34_1":
                total_b_blocks = 17
            elif exp_trial_name == "34_2":
                total_b_blocks = 17

            return total_b_blocks + total_c_blocks

        model_response_dict = {}

        trial_names_list = get_diff_trial_names_list()

        scores = [_diff_block_nums(t_name) for t_name in trial_names_list]

        z_scored_scores = stats.zscore(scores)

        for trial_name in trial_names_list:
            model_response_dict[trial_name] = z_scored_scores[trial_names_list.index(trial_name)]

        return model_response_dict

    @staticmethod
    def get_vlm_scores(out_filepath):
        model_response_dict = {}

        out_file_data = FileUtils.read_csv_file(out_filepath)
        for data in out_file_data:
            model_response_dict[data[0]] = float(data[1])

        return model_response_dict
