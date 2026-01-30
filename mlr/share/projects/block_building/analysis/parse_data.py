import json
import csv

from sqlalchemy import create_engine, MetaData, Table

from mlr.share.projects.block_building.analysis.behavior import DifficultyExperiment, ExperimentType, \
    HandsExperiment, ActionGoalExperiment
from mlr.share.projects.block_building.analysis.model import ModelData, ModelType
from mlr.share.projects.block_building.utils.analysis_utils import TrialKeys
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class Parser:
    @staticmethod
    def _parse_difficulty_participants_db(experiment_type):
        experiment = DifficultyExperiment()

        db_file_path = ""
        if experiment_type == ExperimentType.DIFFICULTY_DIFF:
            db_file_path = PathUtils.join(PathUtils.get_out_human_data_dirpath(), "diff_difficulty_scored.csv")
        elif experiment_type == ExperimentType.DIFFICULTY_TIME:
            db_file_path = PathUtils.join(PathUtils.get_out_human_data_dirpath(), "diff_estimation_scored.csv")
        elif experiment_type == ExperimentType.DIFFICULTY_BUILD:
            db_file_path = PathUtils.join(PathUtils.get_out_human_data_dirpath(), "diff_build_scored.csv")

        with open(db_file_path) as csv_read:
            csv_reader = csv.reader(csv_read, delimiter=",")
            for row in csv_reader:
                trial_name = row[0]
                if trial_name in DifficultyExperiment.DIFF_TRIAL_NAMES:
                    for participant_id, trial_value in enumerate(row[1:], start=1):
                        experiment.add_participant_data(participant_id, trial_name, float(trial_value))

        experiment.apply_exclusion_criteria()
        experiment.z_score_participant_responses(TrialKeys.TRIAL_SLIDER_VALUE)

        return experiment

    @staticmethod
    def parse_participants_db(experiment_type):
        experiment = None
        extra_participants = 0
        if experiment_type == ExperimentType.HANDS:
            experiment = HandsExperiment()
            extra_participants = 7
        elif experiment_type == ExperimentType.ACTION_GOAL:
            experiment = ActionGoalExperiment()
        elif experiment_type == ExperimentType.DIFFICULTY_DIFF:
            return Parser._parse_difficulty_participants_db(experiment_type)
        elif experiment_type == ExperimentType.DIFFICULTY_TIME:
            return Parser._parse_difficulty_participants_db(experiment_type)
        elif experiment_type == ExperimentType.DIFFICULTY_BUILD:
            return Parser._parse_difficulty_participants_db(experiment_type)

        db_path = experiment.get_db_path()
        db_table_name = experiment.get_db_table_name()

        metadata = MetaData()
        engine = create_engine(f"sqlite:///{db_path}")

        table = Table(db_table_name, metadata, autoload_with=engine)
        table_selected = table.select()
        table_rows = engine.connect().execute(table_selected).mappings().all()

        table_data = []
        psiturk_statuses = [3, 4, 5, 7]
        for row in table_rows:
            if row['status'] in psiturk_statuses:
                table_data.append(row['datastring'])

        table_data = [json.loads(part)['data'] for part in table_data]
        if extra_participants > 0:
            table_data = table_data[:-extra_participants]

        for participant_data in table_data:
            experiment.add_participant_data(participant_data)

        experiment.apply_exclusion_criteria()
        experiment.z_score_participant_responses(TrialKeys.TRIAL_SLIDER_VALUE)

        return experiment

    @staticmethod
    def parse_computational_data(experiment_type, model_type):
        model_data = ModelData(model_type)

        model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "overall_out.csv")

        if ExperimentType.DIFFICULTY in experiment_type and model_type == ModelType.PHYSICS_ON:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "diff_phy_ke.csv")
        elif ExperimentType.DIFFICULTY in experiment_type and model_type == ModelType.PHYSICS_ON_SSA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "diff_phy_ssa.csv")
        elif ExperimentType.DIFFICULTY in experiment_type and model_type == ModelType.PHYSICS_OFF:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "diff_no_phy_ke.csv")
        elif ExperimentType.DIFFICULTY in experiment_type and model_type == ModelType.PHYSICS_OFF_SSA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "diff_no_phy_ssa.csv")
        elif ExperimentType.DIFFICULTY in experiment_type and model_type == ModelType.HEURISTICS:
            return model_data.get_diff_heuristics()
        elif ExperimentType.DIFFICULTY in experiment_type and model_type == ModelType.STABILITY:
            return model_data.get_diff_heuristics_stability(0.013)
        elif ExperimentType.DIFFICULTY_DIFF in experiment_type and model_type == ModelType.VLM:
            out_vlm_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_diff_difficulty.csv")
            return model_data.get_vlm_scores(out_vlm_filepath)
        elif ExperimentType.DIFFICULTY_TIME in experiment_type and model_type == ModelType.VLM:
            out_vlm_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_diff_estimation.csv")
            return model_data.get_vlm_scores(out_vlm_filepath)

        if experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.PHYSICS_ON:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "action_goal_phy_ke.csv")
        elif experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.PHYSICS_ON_SSA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "action_goal_phy_ssa.csv")
        elif experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.PHYSICS_OFF:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "action_goal_no_phy_ke.csv")
        elif experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.PHYSICS_OFF_SSA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "action_goal_no_phy_ssa.csv")
        elif experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.HEURISTICS:
            return model_data.get_action_goal_heuristics()
        elif experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.STABILITY:
            return model_data.get_action_goal_heuristics_stability()
        elif experiment_type == ExperimentType.ACTION_GOAL and model_type == ModelType.VLM:
            out_vlm_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_action_goal.csv")
            return model_data.get_vlm_scores(out_vlm_filepath)

        if experiment_type == ExperimentType.HANDS and model_type == ModelType.PHYSICS_ON:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "hands_phy_ke.csv")
        elif experiment_type == ExperimentType.HANDS and model_type == ModelType.PHYSICS_ON_SSA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "hands_phy_ssa.csv")
        elif experiment_type == ExperimentType.HANDS and model_type == ModelType.PHYSICS_OFF:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "hands_no_phy_ke.csv")
        elif experiment_type == ExperimentType.HANDS and model_type == ModelType.PHYSICS_OFF_SSA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "hands_no_phy_ssa.csv")
        elif experiment_type == ExperimentType.HANDS and model_type == ModelType.HEURISTICS:
            return model_data.get_hand_heuristics()
        elif experiment_type == ExperimentType.HANDS and model_type == ModelType.STABILITY:
            return model_data.get_hand_heuristics_stability()
        elif experiment_type == ExperimentType.HANDS and model_type == ModelType.VLM:
            out_vlm_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_hands.csv")
            return model_data.get_vlm_scores(out_vlm_filepath)

        with open(model_data_file_path) as file:
            file_data = csv.reader(file)
            for row in file_data:
                trial_name = row[ModelData.TRIAL_NAME].replace(" ", "_")

                if experiment_type == ExperimentType.HANDS:
                    trial_num_of_hands = int(row[ModelData.TRIAL_NUM_OF_HANDS])
                    if trial_num_of_hands == 1:
                        trial_name += "_1"
                    elif trial_num_of_hands == 2:
                        trial_name += "_2"

                trial_cost = float(row[ModelData.TRIAL_COST])
                trial_energy = float(row[ModelData.TRIAL_ENERGY])

                model_data.add_participant_data(ModelData.COST, trial_name, trial_cost)
                model_data.add_participant_data(ModelData.ENERGY, trial_name, trial_energy)

        model_data.z_score_participant_responses(TrialKeys.TRIAL_SLIDER_VALUE)

        return model_data


if __name__ == '__main__':
    experiment_data = Parser.parse_participants_db(ExperimentType.HANDS)
    experiment_data.apply_exclusion_criteria()
    model_physics_on = Parser.parse_computational_data(ExperimentType.HANDS, ModelType.PHYSICS_ON)
