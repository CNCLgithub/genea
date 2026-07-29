import json
import numpy as np

from sqlalchemy import create_engine, MetaData, Table, select

from mlr.share.projects.navigation.analysis.behavior import NavDiffExperiment, ExperimentType
from mlr.share.projects.navigation.analysis.model import ModelData, ModelType
from mlr.share.projects.navigation.utils.analysis_utils import TrialKeys
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils


class Parser:
    @staticmethod
    def parse_participants_db(experiment_type):
        if experiment_type == ExperimentType.NAV_DIFFICULTY:
            experiment = NavDiffExperiment()
        else:
            Msg.print_error(f"ERROR [Parser]: experiment_type {experiment_type} not supported")
            assert False

        metadata = MetaData()
        engine = create_engine("sqlite:///" + experiment.get_db_path())

        table = Table("genea_nav", metadata, autoload_with=engine)
        s = select(table)

        excluded_unique_ids = []
        excluded_prolific_ids = ["69a9fd65e7edaf2adf755689", "6a00e36e3f9414f0d94977fd"]  # due to time out

        data = []
        psiturk_statuses = [3, 4, 5, 7]  # status codes for successful completion etc.

        with engine.connect() as connection:
            rows = connection.execute(s)
            for row in rows:
                if row.status in psiturk_statuses and row.uniqueid not in excluded_unique_ids:
                    data.append(row.datastring)
                else:
                    print("Excluding subject with ID: " + str(row.uniqueid))

        data = [json.loads(part)['data'] for part in data]

        for subject_data in data:
            experiment.add_participant_data(subject_data, excluded_prolific_ids)

        experiment.apply_exclusion_criteria()
        experiment.z_score_participant_responses(TrialKeys.TRIAL_SLIDER_VALUE)

        return experiment

    @staticmethod
    def parse_model_data(model_type):
        model_data = ModelData(model_type)

        model_data_file_path = PathUtils.join(PathUtils.get_out_dirpath(), "overall_out.csv")
        if model_type == ModelType.NAV_GENEA:
            model_data_file_path = PathUtils.join(PathUtils.get_out_dirpath(), "model_genea.csv")
        if model_type == ModelType.NAV_TAMP:
            model_data_file_path = PathUtils.join(PathUtils.get_out_dirpath(), "model_tamp.csv")
        if model_type == ModelType.STABILITY:
            out_stability_filepath = PathUtils.join(PathUtils.get_out_dirpath(), "model_stability.csv")
            return model_data.get_stability_scores(out_stability_filepath)
        elif model_type == ModelType.VLM:
            out_vlm_filepath = PathUtils.join(PathUtils.get_out_vlm_dirpath(), "model_vlm.csv")
            return model_data.get_vlm_scores(out_vlm_filepath)

        stim_str_dict = {}
        for data in FileUtils.read_csv_file(model_data_file_path)[1:]:
            stimulus_name = data[0]
            stim_str = int(stimulus_name.split("_")[1])
            stim_var = int(stimulus_name.split("_")[2])
            if stim_var == 3:  # since these were excluded in the behavior experiment
                continue

            stim_str = f"{stim_str}_{stim_var}"

            is_success = int(data[1])
            iter_num = int(data[2])
            cost_ke = float(data[3])
            cost_crocoddyl = float(data[4])
            sym_len = int(data[6])
            if stim_str not in stim_str_dict:
                stim_str_dict[stim_str] = {}
                stim_str_dict[stim_str][iter_num] = {}
            if iter_num not in stim_str_dict[stim_str]:
                stim_str_dict[stim_str][iter_num] = {}
            if ModelData.TRIAL_SUCCESS not in stim_str_dict[stim_str][iter_num]:
                stim_str_dict[stim_str][iter_num][ModelData.TRIAL_SUCCESS] = []
            if ModelData.COST_KE not in stim_str_dict[stim_str][iter_num]:
                stim_str_dict[stim_str][iter_num][ModelData.COST_KE] = []
            if ModelData.COST_CROCODDYL not in stim_str_dict[stim_str][iter_num]:
                stim_str_dict[stim_str][iter_num][ModelData.COST_CROCODDYL] = []
            if ModelData.COST_SYM_LEN not in stim_str_dict[stim_str][iter_num]:
                stim_str_dict[stim_str][iter_num][ModelData.COST_SYM_LEN] = []

            stim_str_dict[stim_str][iter_num][ModelData.TRIAL_SUCCESS].append(is_success)
            stim_str_dict[stim_str][iter_num][ModelData.COST_KE].append(cost_ke)
            stim_str_dict[stim_str][iter_num][ModelData.COST_CROCODDYL].append(cost_crocoddyl)
            stim_str_dict[stim_str][iter_num][ModelData.COST_SYM_LEN].append(sym_len)

        cost_ke_dict = {}
        cost_crocoddyl_dict = {}
        cost_sym_len_dict = {}
        for stim_str, stim_iter_dict in stim_str_dict.items():
            for iter_num, trial_dict in stim_iter_dict.items():
                trial_success = np.asarray(trial_dict[ModelData.TRIAL_SUCCESS])
                if sum(trial_success) == 0:
                    continue

                indices = np.where(trial_success == 1)

                cost_ke = np.min(np.asarray(trial_dict[ModelData.COST_KE])[indices])
                cost_crocoddyl = np.min(np.asarray(trial_dict[ModelData.COST_CROCODDYL])[indices])
                cost_sym_len = np.min(np.asarray(trial_dict[ModelData.COST_SYM_LEN])[indices])

                if cost_ke > 10 ** 7:
                    continue

                if stim_str not in cost_ke_dict:
                    cost_ke_dict[stim_str] = []
                    cost_crocoddyl_dict[stim_str] = []
                    cost_sym_len_dict[stim_str] = []
                cost_ke_dict[stim_str].append(cost_ke)
                cost_crocoddyl_dict[stim_str].append(cost_crocoddyl)
                cost_sym_len_dict[stim_str].append(cost_sym_len)

        ke_dict = {}
        croc_dict = {}
        s_len_dict = {}
        for stim_str in cost_ke_dict.keys():
            stim_num = int(stim_str.split("_")[0])

            cost_ke_dict[stim_str] = cost_ke_dict[stim_str][:60]
            cost_crocoddyl_dict[stim_str] = cost_crocoddyl_dict[stim_str][:60]
            cost_sym_len_dict[stim_str] = cost_sym_len_dict[stim_str][:60]

            if stim_num not in ke_dict:
                ke_dict[stim_num] = cost_ke_dict[stim_str]
                croc_dict[stim_num] = cost_crocoddyl_dict[stim_str]
                s_len_dict[stim_num] = cost_sym_len_dict[stim_str]
            else:
                ke_dict[stim_num] = [(a + b) / 2 for a, b in zip(ke_dict[stim_num], cost_ke_dict[stim_str])]
                croc_dict[stim_num] = [(a + b) / 2 for a, b in zip(croc_dict[stim_num], cost_crocoddyl_dict[stim_str])]
                s_len_dict[stim_num] = [(a + b) / 2 for a, b in zip(s_len_dict[stim_num], cost_sym_len_dict[stim_str])]

        for stim_num in ke_dict.keys():
            for i in range(len(ke_dict[stim_num])):
                model_data.add_participant_data(ModelData.COST_KE, stim_num, ke_dict[stim_num][i])
                model_data.add_participant_data(ModelData.COST_CROCODDYL, stim_num, croc_dict[stim_num][i])
                model_data.add_participant_data(ModelData.COST_SYM_LEN, stim_num, s_len_dict[stim_num][i])

        stim_str_list = sorted(list(cost_ke_dict.keys()), key=lambda x: int(x.split("_")[1]))
        stim_str_list = sorted(stim_str_list, key=lambda x: int(x.split("_")[0]))
        for stim_str in stim_str_list:
            ke_avg = np.mean(cost_ke_dict[stim_str]).item()
            ke_std = np.std(cost_ke_dict[stim_str]).item()
            print(f"Stim: {stim_str}, mean: {ke_avg}, std: {ke_std}")

        model_data.z_score_participant_responses(TrialKeys.TRIAL_SLIDER_VALUE)

        return model_data
