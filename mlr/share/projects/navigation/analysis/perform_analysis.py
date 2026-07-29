import click
import numpy as np
from scipy.stats import binomtest

from mlr.share.projects.navigation.analysis.behavior import ExperimentType
from mlr.share.projects.navigation.analysis.model import ModelType, ModelData
from mlr.share.projects.navigation.analysis.parser import Parser
from mlr.share.projects.navigation.utils.analysis_utils import TrialKeys
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import AnalysisConfig
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.plot_utils import PlotUtils


class JSONToHTML:
    PHASE_NAMES = {
        "instructions": "Instructions",
        "practice": "Practice",
        "trial": "Trial"
    }

    TYPE_NAMES = {
        "text": "Text",
        "text_img": "Text + Image",
        "bool_t": "Text",
        "bool_f": "Text"
    }

    EXPECTED_RESPONSES = {
        "text": "N/A",
        "text_img": "An integer (1–100)",
        "bool_t": "True",
        "bool_f": "False"
    }

    @staticmethod
    def _generate_rows(data):
        rows = []

        for phase, items in data.items():
            for item in items:
                item_type = item.get("type", "")
                prompt = "".join(item.get("prompt", []))
                rows.append(f"""
                            <tr>
                                <td>{JSONToHTML.PHASE_NAMES.get(phase, phase)}</td>
                                <td>{JSONToHTML.TYPE_NAMES.get(item_type, item_type)}</td>
                                <td>{prompt}</td>
                                <td>{JSONToHTML.EXPECTED_RESPONSES.get(item_type, "N/A")}</td>
                            </tr>""")

        return "\n".join(rows)

    @staticmethod
    def generate_html(json_filepath, output_filepath):
        data = FileUtils.read_json_file(json_filepath)
        rows = JSONToHTML._generate_rows(data)
        template = FileUtils.read_file(PathUtils.join(PathUtils.get_vlm_files_dirpath(), "template.html"))
        FileUtils.write_to_file(output_filepath, template.replace("{rows}", rows))


def run_split_half_bootstrap(behavior_data):
    behavior_trial_key = TrialKeys.TRIAL_Z_SCORE_VALUE
    trial_names_list = behavior_data.get_all_trial_names()

    bootstrap_cor_list = []

    participant_id_list = behavior_data.get_participant_id_list()

    for _ in range(1000):
        np.random.shuffle(participant_id_list)

        sampled_ids1 = participant_id_list[:len(participant_id_list) // 2]
        sampled_ids2 = participant_id_list[len(participant_id_list) // 2:]
        behavior_responses_dict1 = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids1)
        behavior_responses_dict2 = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids2)

        behavior_responses_list1 = []
        behavior_responses_list2 = []
        for trial_name in trial_names_list:
            behavior_responses_list1.append(behavior_responses_dict1[trial_name])
            behavior_responses_list2.append(behavior_responses_dict2[trial_name])

        if AnalysisConfig.ANALYSIS_PEARSON:
            bootstrap_cor_list.append(ComputeUtils.get_pearsonr(behavior_responses_list1, behavior_responses_list2))
        else:
            bootstrap_cor_list.append(ComputeUtils.get_spearmanr(behavior_responses_list1, behavior_responses_list2))

    correlation_value = np.mean(bootstrap_cor_list).item()
    Msg.print_warn(f"HUMAN vs HUMAN 50/50 = {correlation_value}")

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
        sampled_ids = np.random.choice(list(participant_id_list), len(participant_id_list), replace=True)
        behavior_responses_dict = behavior_data.get_mean_trial_responses_dict(behavior_trial_key, sampled_ids)

        behavior_responses_list = []
        for trial_name in trial_names_list:
            behavior_responses_list.append(behavior_responses_dict[trial_name])

        for model_name in model_names_list:
            model_responses_list = model_response_list_dict[model_name]
            if AnalysisConfig.ANALYSIS_PEARSON:
                r = ComputeUtils.get_pearsonr(behavior_responses_list, model_responses_list)
            else:
                r = ComputeUtils.get_spearmanr(behavior_responses_list, model_responses_list)
            bootstrap_cor_list[model_name].append(r)

    for model_name in model_names_list:
        correlation_value = np.mean(bootstrap_cor_list[model_name]).item()
        Msg.print_warn(f"BOOTSTRAP HUMAN vs MODEL {model_name} = {correlation_value}")

    if save_filepath:
        write_list = []
        for model in model_names_list:
            write_list.append(np.mean(bootstrap_cor_list[model]).item())
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

    if AnalysisConfig.ANALYSIS_PEARSON:
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


def draw_model_vs_behavior_bar_plots(experiment_type, behavior_data, model_response_dict_dict, save_path, do_rt=False):
    corr_dict = run_behavior_vs_model_bootstrap(experiment_type, behavior_data, model_response_dict_dict, do_rt=do_rt)
    PlotUtils.draw_bootstrap_bar_plot(list(model_response_dict_dict.keys()), corr_dict, [-0.2, 1.2], save_path)


def draw_material_bar_plots(behavior_data, model_response_dict):
    stone_trails_list = [1, 4, 6, 8, 10, 15, 3]
    woody_trails_list = [2, 5, 7, 9, 11, 16, 21]
    behavior_response_dict = behavior_data.get_all_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)
    # behavior_response_dict = behavior_data.get_mean_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)

    trial_names_list = stone_trails_list + woody_trails_list

    behavior_responses_list = []
    model_responses_list = []

    for trial_name in trial_names_list:
        behavior_responses_list.append(behavior_response_dict[trial_name])
        model_responses_list.append(model_response_dict[trial_name])

    b_stone = np.mean(behavior_responses_list[:len(stone_trails_list)], axis=0)
    b_woody = np.mean(behavior_responses_list[len(stone_trails_list):], axis=0)
    m_stone = np.mean(model_responses_list[:len(stone_trails_list)], axis=0)
    m_woody = np.mean(model_responses_list[len(stone_trails_list):], axis=0)

    # b_stone = behavior_responses_list[:len(stone_trails_list)]
    # b_woody = behavior_responses_list[len(stone_trails_list):]
    # m_stone = model_responses_list[:len(stone_trails_list)]
    # m_woody = model_responses_list[len(stone_trails_list):]

    bt, bp = ComputeUtils.compute_paired_t_test(b_stone, b_woody)
    mt, mp = ComputeUtils.compute_paired_t_test(m_stone, m_woody)

    print(f"Behavior t-test: {bt:.2f} ({bp:.3f})")
    print(f"Model t-test: {mt:.2f} ({mp:.3f})")

    x_values_list = [i for i in [0, 1] for _ in range(len(b_stone))]
    y_values_list = list(b_stone) + list(b_woody)
    save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), "material_behavior.pdf")
    PlotUtils.draw_strip_plot(x_values_list, y_values_list, False, False, save_path=save_filepath)

    x_values_list = [i for i in [0, 1] for _ in range(len(m_stone))]
    y_values_list = list(m_stone) + list(m_woody)
    save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), "material_model.pdf")
    PlotUtils.draw_strip_plot(x_values_list, y_values_list, False, False, save_path=save_filepath)


def draw_behavior_plot(behavior_data):
    behavior_response_dict = behavior_data.get_all_trial_responses_dict(TrialKeys.TRIAL_SLIDER_VALUE)
    behavior_responses_list = []
    for trial_name in behavior_response_dict.keys():
        behavior_responses_list.append(behavior_response_dict[trial_name])

    x_values_list = [label for label, values in list(behavior_response_dict.items()) for _ in values]
    y_values_list = [v for values in behavior_response_dict.values() for v in values]
    save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), "behavior.pdf")
    PlotUtils.draw_strip_plot(x_values_list, y_values_list, True, False, False, save_filepath)


def test_binomial(behavior_response_dict, model_response_dict):
    stone_trails_list = [1, 4, 6, 8, 10, 15, 3]
    woody_trails_list = [2, 5, 7, 9, 11, 16, 21]

    trial_names_list = stone_trails_list + woody_trails_list

    behavior_responses_list = []
    model_responses_list = []

    for trial_name in trial_names_list:
        behavior_responses_list.append(sorted(behavior_response_dict[trial_name]))
        model_responses_list.append(sorted(model_response_dict[trial_name]))

    b_stone = behavior_responses_list[:len(stone_trails_list)]
    b_woody = behavior_responses_list[len(stone_trails_list):]
    m_stone = model_responses_list[:len(stone_trails_list)]
    m_woody = model_responses_list[len(stone_trails_list):]

    wins = 0
    total = 0

    for stone_trial, woody_trial in zip(b_stone, b_woody):
        for s in stone_trial:
            for w in woody_trial:
                if w > s:
                    wins += 1
                total += 1

    result = binomtest(wins, total, p=0.5, alternative="greater")
    print(wins, total, result.pvalue)

    wins = 0
    total = 0

    for stone_trial, woody_trial in zip(m_stone, m_woody):
        for s in stone_trial:
            for w in woody_trial:
                if w > s:
                    wins += 1
                total += 1

    result = binomtest(wins, total, p=0.5, alternative="greater")
    print(wins, total, result.pvalue)


def generate_vlm_prompt_html():
    for filename in FileUtils.get_files_in_directory(PathUtils.get_vlm_json_files_dirpath()):
        if not filename.endswith(".json"):
            continue
        filename = FileUtils.get_basename(filename, False)[0]
        json_filepath = PathUtils.join(PathUtils.get_vlm_json_files_dirpath(), filename + ".json")
        html_filepath = PathUtils.join(PathUtils.get_vlm_json_files_dirpath(), f"supp_vlm_{filename}.html")
        JSONToHTML().generate_html(json_filepath, html_filepath)


def run_analysis(experiment_type):
    Msg.print_success(f"{experiment_type}")

    behavior = Parser.parse_participants_db(ExperimentType.NAV_DIFFICULTY)

    genea = Parser.parse_model_data(ModelType.NAV_GENEA)
    tamp = Parser.parse_model_data(ModelType.NAV_TAMP)

    genea_all_ke_dict = genea.get_all_model_responses_list(ModelData.COST_KE)

    genea_ke_dict = genea.get_model_response_list(ModelData.COST_KE)
    tamp_ke_dict = tamp.get_model_response_list(ModelData.COST_KE)
    genea_croc_dict = genea.get_model_response_list(ModelData.COST_CROCODDYL)
    genea_sym_dict = genea.get_model_response_list(ModelData.COST_SYM_LEN)

    stability_dict = Parser.parse_model_data(ModelType.STABILITY)
    vlm_dict = Parser.parse_model_data(ModelType.VLM)

    model_pref_dict_dict = {
        "genea_ke": genea_ke_dict,
        "tamp_ke": tamp_ke_dict,
        "genea_crocoddyl": genea_croc_dict,
        "genea_sym": genea_sym_dict,
        "stability": stability_dict,
        "vlm": vlm_dict
    }

    draw_behavior_plot(behavior)

    save_filepath = PathUtils.join(PathUtils.get_out_plots_dirpath(), str(experiment_type).lower() + "_bar.pdf")
    draw_model_vs_behavior_bar_plots(experiment_type, behavior, model_pref_dict_dict, save_filepath)

    run_behavior_vs_model_correlation(behavior, genea_ke_dict, "model")
    draw_material_bar_plots(behavior, genea_all_ke_dict)


@click.command()
@click.option("-d", "--nav_diff", default=False, is_flag=True, help="analyze action-goal inference data")
def main(nav_diff):
    if nav_diff:
        run_analysis(ExperimentType.NAV_DIFFICULTY)
        return

    # generate_vlm_prompt_html()


if __name__ == '__main__':
    main()
