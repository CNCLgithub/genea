import click
import numpy as np
import random
import time

from mlr.share.projects.block_building.analysis.parse_data import ExperimentType, Parser, ModelType, ModelData
from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.core_utils import ConfigUtils
from mlr.share.projects.block_building.utils.file_utils import FileUtils
from mlr.share.projects.block_building.utils.path_utils import PathUtils
from mlr.share.projects.block_building.utils.plot_utils import PlotUtils
from mlr.share.projects.block_building.utils.analysis_utils import TrialKeys
from mlr.share.projects.block_building.utils.msg_utils import Msg


def run_split_half_bootstrap(behavior_data):
    behavior_trial_key = TrialKeys.TRIAL_Z_SCORE_VALUE
    trial_names_list = behavior_data.get_all_trial_names()

    bootstrap_cor_list = []

    participant_id_list = behavior_data.get_participant_id_list()

    for _ in range(1000):
        random.shuffle(participant_id_list)

        sampled_ids1 = participant_id_list[:len(participant_id_list) // 2]
        sampled_ids2 = participant_id_list[len(participant_id_list) // 2:]
        behavior_responses_dict1 = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids1)
        behavior_responses_dict2 = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids2)

        behavior_responses_list1 = []
        behavior_responses_list2 = []
        for trial_name in trial_names_list:
            behavior_responses_list1.append(behavior_responses_dict1[trial_name])
            behavior_responses_list2.append(behavior_responses_dict2[trial_name])

        if ConfigUtils.ANALYSIS_PEARSON:
            bootstrap_cor_list.append(ComputeUtils.get_pearsonr(behavior_responses_list1, behavior_responses_list2))
        else:
            bootstrap_cor_list.append(ComputeUtils.get_spearmanr(behavior_responses_list1, behavior_responses_list2))

    correlation_value = np.mean(bootstrap_cor_list)
    Msg.print_warn("HUMAN vs HUMAN 50/50 = " + str(correlation_value))

    return bootstrap_cor_list


def run_leave_one_out_bootstrap(behavior_data):
    behavior_trial_key = TrialKeys.TRIAL_Z_SCORE_VALUE
    trial_names_list = behavior_data.get_all_trial_names()

    bootstrap_cor_list = []

    participant_id_list = behavior_data.get_participant_id_list()

    for _ in range(1000):
        random.shuffle(participant_id_list)

        sampled_ids1 = participant_id_list[:-1]
        sampled_ids2 = participant_id_list[-1]
        behavior_responses_dict1 = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids1)
        behavior_responses_dict2 = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids2)

        behavior_responses_list1 = []
        behavior_responses_list2 = []
        for trial_name in trial_names_list:
            behavior_responses_list1.append(behavior_responses_dict1[trial_name])
            behavior_responses_list2.append(behavior_responses_dict2[trial_name])

        if ConfigUtils.ANALYSIS_PEARSON:
            bootstrap_cor_list.append(ComputeUtils.get_pearsonr(behavior_responses_list1, behavior_responses_list2))
        else:
            bootstrap_cor_list.append(ComputeUtils.get_spearmanr(behavior_responses_list1, behavior_responses_list2))

    correlation_value = np.mean(bootstrap_cor_list)
    Msg.print_warn("HUMAN vs HUMAN ALL-1/1 = " + str(correlation_value))

    return bootstrap_cor_list


def run_behavior_vs_model_bootstrap(exp_type, behavior_data, model_response_dict_dict, save_filepath=None, do_rt=False):
    Msg.print_info("Running bootstrap analysis for experiment type: " + str(exp_type))
    if do_rt:
        behavior_trial_key = TrialKeys.TRIAL_REACTION_TIME
    else:
        behavior_trial_key = TrialKeys.TRIAL_SLIDER_VALUE

    model_names_list = list(model_response_dict_dict.keys())
    trial_names_list = behavior_data.get_all_trial_names()

    model_response_list_dict = {}
    for model_name in model_names_list:
        model_responses_list = []
        model_response_dict = model_response_dict_dict[model_name]
        for trial_name in trial_names_list:
            model_responses_list.append(model_response_dict[trial_name])
        model_response_list_dict[model_name] = model_responses_list

    bootstrap_cor_list = {model_name: [] for model_name in model_names_list}

    participant_id_list = behavior_data.get_participant_id_list()

    for _ in range(1000):
        sampled_ids = random.choices(list(participant_id_list), k=len(participant_id_list))
        behavior_responses_dict = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids)

        behavior_responses_list = []
        for trial_name in trial_names_list:
            behavior_responses_list.append(behavior_responses_dict[trial_name])

        for model_name in model_names_list:
            model_responses_list = model_response_list_dict[model_name]
            if ConfigUtils.ANALYSIS_PEARSON:
                r = ComputeUtils.get_pearsonr(behavior_responses_list, model_responses_list)
            else:
                r = ComputeUtils.get_spearmanr(behavior_responses_list, model_responses_list)
            bootstrap_cor_list[model_name].append(r)

    for model_name in model_names_list:
        correlation_value = np.mean(bootstrap_cor_list[model_name])
        Msg.print_warn("BOOTSTRAP HUMAN vs MODEL " + model_name + " = " + str(correlation_value))

    if save_filepath:
        write_list = [ConfigUtils.TEMP_DEFAULT]
        for model in model_names_list:
            write_list.append(np.mean(bootstrap_cor_list[model]))
        FileUtils.write_row_to_file(save_filepath, write_list)

    return bootstrap_cor_list


def run_reaction_time_bootstrap(experiment_type, behavior_data, model_response_dict_dict, save_filepath=None):
    Msg.print_info("Running bootstrap analysis for experiment type: " + str(experiment_type))

    behavior_trial_key = TrialKeys.TRIAL_REACTION_TIME
    model_names_list = list(model_response_dict_dict.keys())
    trial_names_list = behavior_data.get_all_trial_names()

    model_response_list_dict = {}
    for model_name in model_names_list:
        model_responses_list = []
        model_response_dict = model_response_dict_dict[model_name]
        for trial_name in trial_names_list:
            model_responses_list.append(model_response_dict[trial_name])
        model_response_list_dict[model_name] = model_responses_list

    bootstrap_cor_list = {model_name: [] for model_name in model_names_list}

    participant_id_list = behavior_data.get_participant_id_list()

    for _ in range(1000):
        sampled_ids = random.choices(list(participant_id_list), k=len(participant_id_list))
        behavior_responses_dict = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids)

        behavior_responses_list = []
        for trial_name in trial_names_list:
            behavior_responses_list.append(behavior_responses_dict[trial_name])

        for model_name in model_names_list:
            model_responses_list = model_response_list_dict[model_name]
            if ConfigUtils.ANALYSIS_PEARSON:
                r = ComputeUtils.get_pearsonr(behavior_responses_list, model_responses_list)
            else:
                r = ComputeUtils.get_spearmanr(behavior_responses_list, model_responses_list)
            bootstrap_cor_list[model_name].append(r)

    for model_name in model_names_list:
        correlation_value = np.mean(bootstrap_cor_list[model_name])
        Msg.print_warn("BOOTSTRAP HUMAN vs MODEL " + model_name + " = " + str(correlation_value))

    if save_filepath:
        write_list = [ConfigUtils.TEMP_DEFAULT]
        for model in model_names_list:
            write_list.append(np.mean(bootstrap_cor_list[model]))
        FileUtils.write_row_to_file(save_filepath, write_list)

    return bootstrap_cor_list


def run_behavior_vs_model_correlation(behavior_data, model_response_dict, model_name=None, do_plot=True, do_z=False):
    behavior_response_dict = behavior_data.get_mean_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)

    trial_names_list = behavior_data.get_all_trial_names()

    behavior_responses_list = []
    model_responses_list = []

    for trial_name in trial_names_list:
        behavior_responses_list.append(behavior_response_dict[trial_name])
        model_responses_list.append(model_response_dict[trial_name])

    if ConfigUtils.ANALYSIS_PEARSON:
        correlation_value, p_value = ComputeUtils.get_pearsonr(behavior_responses_list, model_responses_list, True)
    else:
        correlation_value, p_value = ComputeUtils.get_spearmanr(behavior_responses_list, model_responses_list, True)

    if model_name is None:
        model_name = "model"

    if do_plot:
        if do_z:
            model_responses_list = ComputeUtils.zscore_list(model_responses_list)

        save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), model_name + ".pdf")
        PlotUtils.draw_scatter_plot(model_responses_list, behavior_responses_list,
                                    f"behavior vs {model_name}", model_name, "behavior",
                                    save_path=save_filepath, annot=trial_names_list)

    Msg.print_warn("HUMAN vs MODEL " + model_name + " = " + str(correlation_value) + ", " + str(p_value))


def run_reaction_time_correlation(behavior_data, model_response_dict, model_name=None, do_plot=True, do_z=False):
    behavior_pref_dict = behavior_data.get_mean_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)
    behavior_resp_dict = behavior_data.get_mean_trial_responses_dict(TrialKeys.TRIAL_REACTION_TIME)

    trial_names_list = behavior_data.get_all_trial_names()

    behavior_pref_list = []
    behavior_rt_list = []
    model_pref_list = []

    for trial_name in trial_names_list:
        behavior_pref_list.append(behavior_pref_dict[trial_name])
        behavior_rt_list.append(behavior_resp_dict[trial_name])
        model_pref_list.append(model_response_dict[trial_name])

    rt_pref_resid = ComputeUtils.get_res_partial_regression(behavior_rt_list, behavior_pref_list)
    model_pref_resid = ComputeUtils.get_res_partial_regression(model_pref_list, behavior_pref_list)
    ComputeUtils.get_res_partial_regression(rt_pref_resid[0], model_pref_resid[0])

    if ConfigUtils.ANALYSIS_PEARSON:
        correlation_value, p_value = ComputeUtils.get_pearsonr(behavior_rt_list, model_pref_list, True)
    else:
        correlation_value, p_value = ComputeUtils.get_spearmanr(behavior_rt_list, model_pref_list, True)

    if model_name is None:
        model_name = "model"

    if do_plot:
        if do_z:
            model_pref_list = ComputeUtils.zscore_list(model_pref_list)

        save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), model_name + "_rt.pdf")
        PlotUtils.draw_scatter_plot(model_pref_list, behavior_rt_list,
                                    f"behavior vs {model_name}", model_name, "behavior",
                                    save_path=save_filepath, annot=trial_names_list)

    Msg.print_warn("HUMAN vs MODEL " + model_name + " = " + str(correlation_value) + ", " + str(p_value))


def draw_model_vs_behavior_bar_plots(experiment_type, behavior_data, model_response_dict_dict, save_path, do_rt=False):
    corr_dict = run_behavior_vs_model_bootstrap(experiment_type, behavior_data, model_response_dict_dict, do_rt=do_rt)
    floor_ceiling_list = [-0.1, 1.1]
    PlotUtils.draw_multiple_bar_plots(list(model_response_dict_dict.keys()),
                                      corr_dict,
                                      floor_ceiling_list, save_path=save_path)


def draw_box_plot(behavior_data, model_response_dict):
    behavior_response_dict = behavior_data.get_all_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)

    trial_names_list = behavior_data.get_all_trial_names()

    behavior_x_list = []
    behavior_responses_list = []
    model_responses_list = []
    for trial_name in trial_names_list:
        behavior_x_list.extend([trial_name] * len(behavior_response_dict[trial_name]))
        behavior_responses_list.extend(behavior_response_dict[trial_name])
        model_responses_list.append(model_response_dict[trial_name])

    model_responses_list = [(p * 100) for p in model_responses_list]

    PlotUtils.draw_strip_plot(behavior_x_list, behavior_responses_list, trial_names_list, model_responses_list)


def run_analysis(experiment_type):
    print(experiment_type)

    behavior_data = Parser.parse_participants_db(experiment_type)
    phys_model_data = Parser.parse_computational_data(experiment_type, ModelType.PHYSICS_ON)
    no_phys_model_data = Parser.parse_computational_data(experiment_type, ModelType.PHYSICS_OFF)

    phy_energy_pref_dict = phys_model_data.get_model_response_list(experiment_type, ModelData.ENERGY)
    no_phy_energy_pref_dict = no_phys_model_data.get_model_response_list(experiment_type, ModelData.ENERGY)

    phy_energy_rt_dict = phys_model_data.get_model_response_list(experiment_type, ModelData.ENERGY, True)
    no_phy_energy_rt_dict = no_phys_model_data.get_model_response_list(experiment_type, ModelData.ENERGY, True)
    phy_cost_rt_dict = phys_model_data.get_model_response_list(experiment_type, ModelData.COST, True)
    no_phy_cost_rt_dict = no_phys_model_data.get_model_response_list(experiment_type, ModelData.COST, True)

    phy_cost_pref_dict = phys_model_data.get_model_response_list(experiment_type, ModelData.COST)
    no_phy_cost_pref_dict = no_phys_model_data.get_model_response_list(experiment_type, ModelData.COST)

    heuristics_pref_dict = Parser.parse_computational_data(experiment_type, ModelType.HEURISTICS)
    stability_pref_dict = Parser.parse_computational_data(experiment_type, ModelType.STABILITY)

    phys_ssa_model_data = Parser.parse_computational_data(experiment_type, ModelType.PHYSICS_ON_SSA)
    no_phys_ssa_model_data = Parser.parse_computational_data(experiment_type, ModelType.PHYSICS_OFF_SSA)
    phy_ssa_pref_dict = phys_ssa_model_data.get_model_response_list(experiment_type, ModelData.ENERGY)
    no_phy_ssa_pref_dict = no_phys_ssa_model_data.get_model_response_list(experiment_type, ModelData.ENERGY)
    phy_ssa_rt_dict = phys_ssa_model_data.get_model_response_list(experiment_type, ModelData.ENERGY, True)
    no_phy_ssa_rt_dict = no_phys_ssa_model_data.get_model_response_list(experiment_type, ModelData.ENERGY, True)

    vlm_pref_dict = Parser.parse_computational_data(experiment_type, ModelType.VLM)

    model_pref_dict_dict = {
        "phy_on_energy": phy_energy_pref_dict,
        "phy_off_energy": no_phy_energy_pref_dict,
        "phy_on_cost": phy_cost_pref_dict,
        "phy_off_cost": no_phy_cost_pref_dict,
        "phy_on_ssa": phy_ssa_pref_dict,
        "phy_off_ssa": no_phy_ssa_pref_dict,
        "stability": stability_pref_dict,
        "heuristic": heuristics_pref_dict,
        "vlm": vlm_pref_dict
    }

    # ----------- uncomment when running temperature analysis -----------
    save_filepath = PathUtils.join(PathUtils.get_out_files_dirpath(), "boot_" + str(experiment_type).lower() + ".csv")
    run_behavior_vs_model_bootstrap(experiment_type, behavior_data, model_pref_dict_dict, save_filepath)

    # ----------- produces bar graphs -----------
    save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), str(experiment_type).lower() + "_bar.pdf")
    draw_model_vs_behavior_bar_plots(experiment_type, behavior_data, model_pref_dict_dict, save_filepath)

    # ----------- produces correlation scatter graphs -----------
    run_behavior_vs_model_correlation(behavior_data, phy_energy_pref_dict, experiment_type + "_" + "phy_on_energy")

    # ----------- produces reaction time graphs -----------
    if ExperimentType.DIFFICULTY not in experiment_type:
        behavior_dict = behavior_data.get_mean_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)

        model_pref_dict_dict = {
            "behavior": behavior_dict,
            "rt_phy_on": phy_energy_rt_dict,
            "rt_phy_off": no_phy_energy_rt_dict,
            "rt_cost_phy_on": phy_cost_rt_dict,
            "rt_cost_phy_off": no_phy_cost_rt_dict,
            "rt_ssa_phy_on": phy_ssa_rt_dict,
            "rt_ssa_phy_off": no_phy_ssa_rt_dict,
        }

        save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), str(experiment_type).lower() + "_bar_rt.pdf")
        draw_model_vs_behavior_bar_plots(experiment_type, behavior_data, model_pref_dict_dict, save_filepath, True)

        run_reaction_time_correlation(behavior_data, phy_energy_rt_dict, experiment_type + "_" + "phy_on_rt")

    time.sleep(1)
    Msg.println_normal("#################################################")


@click.command()
@click.option("-ag", "--action_goal_inference", default=False, is_flag=True, help="analyze action-goal inference data")
@click.option("-e", "--effector_inference", default=False, is_flag=True, help="analyze hands data")
@click.option("-d", "--difficulty", default=False, is_flag=True, help="analyze difficulty data")
@click.option("-t", "--temperature", default=False, is_flag=True, help="analyze effect of temperature parameter")
def main(action_goal_inference, effector_inference, difficulty, temperature):
    Msg.print_warn("Running analysis...")

    if action_goal_inference:
        run_analysis(ExperimentType.ACTION_GOAL)
        return

    if effector_inference:
        run_analysis(ExperimentType.HANDS)
        return

    if difficulty:
        run_analysis(ExperimentType.DIFFICULTY_DIFF)
        run_analysis(ExperimentType.DIFFICULTY_TIME)
        run_analysis(ExperimentType.DIFFICULTY_BUILD)
        return

    if temperature:
        for temperature in np.linspace(1, 11, 101):
            Msg.print_warn("----------" + str(temperature) + "----------")
            ConfigUtils.TEMP_DEFAULT = temperature
            ConfigUtils.TEMP_ENERGY_AG = temperature
            ConfigUtils.TEMP_ENERGY_E = temperature
            ConfigUtils.TEMP_COST_AG = temperature
            ConfigUtils.TEMP_COST_E = temperature
            ConfigUtils.TEMP_SSA_AG = temperature
            ConfigUtils.TEMP_SSA_E = temperature

            run_analysis(ExperimentType.ACTION_GOAL)
            run_analysis(ExperimentType.HANDS)
        return


if __name__ == '__main__':
    main()
