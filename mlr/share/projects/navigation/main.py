import click

from mlr.share.projects.navigation.model.planner import NavModel
from mlr.share.projects.navigation.model.stimuli import StimuliDiff
from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.config_utils import CoreConfig
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class ModelRunner:
    @staticmethod
    def run_diff_planner(stim_index):
        stim_diff = StimuliDiff()
        stim_dir_list = FileUtils.get_dir_list_in_directory(stim_diff.get_stimuli_set_dirpath())
        stim_dir_list.sort(key=lambda x: int(x.split("/")[-1].split("_")[1]))

        stim_dirname = FileUtils.get_basename(stim_dir_list[stim_index])
        out_filepath = PathUtils.join(PathUtils.get_out_model_dirpath(), f"{stim_dirname}.csv")

        Msg.print_success("Running model on stimulus: " + stim_dirname)

        for run_num in range(CoreConfig.EXP_ITERATIONS):
            nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliDiff(), stim_dirname))
            nav_model.run_planner()
            nav_model.print_possible_paths()
            nav_model.save_state_to_file(out_filepath, run_num + 1)

    @staticmethod
    def run_diff_stability(stim_index):
        stim_diff = StimuliDiff()
        stim_dir_list = FileUtils.get_dir_list_in_directory(stim_diff.get_stimuli_set_dirpath())
        stim_dir_list.sort(key=lambda x: int(x.split("/")[-1].split("_")[1]))

        stim_dirname = FileUtils.get_basename(stim_dir_list[stim_index])
        out_filepath = PathUtils.join(PathUtils.get_out_model_dirpath(), f"{stim_dirname}_stability.csv")

        Msg.print_success("Running stability heuristic on stimulus: " + stim_dirname)

        nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliDiff(), stim_dirname))
        nav_model.run_stability_heuristic(out_filepath)

    @staticmethod
    def combine_results(stimuli, out_filename):
        stim_dir_list = FileUtils.get_dir_list_in_directory(stimuli.get_stimuli_set_dirpath())
        stim_dir_list.sort(key=lambda x: int(x.split("/")[-1].split("_")[1]))

        out_filepath = PathUtils.join(PathUtils.get_out_dirpath(), out_filename)

        for stim_index, stim_dirpath in enumerate(stim_dir_list):
            stim_dirname = FileUtils.get_basename(stim_dirpath)
            stim_filepath = PathUtils.join(PathUtils.get_out_dirpath(), f"{stim_dirname}.csv")
            if "stability" in out_filename:
                stim_filepath = PathUtils.join(PathUtils.get_out_dirpath(), f"{stim_dirname}_stability.csv")

            if not FileUtils.is_file(stim_filepath):
                continue

            stim_data = FileUtils.read_csv_file(stim_filepath)
            if stim_index > 0:
                stim_data = stim_data[1:]

            for data in stim_data:
                FileUtils.write_row_to_file(out_filepath, data)


@click.command()
@click.option("-si", "--stim_index", type=click.STRING, help="the index of the stimulus to run")
def main(stim_index):
    # ModelRunner.run_diff_planner(int(stim_index))
    # ModelRunner.combine_results(StimuliDiff(), "model_genea.csv")

    ModelRunner.run_diff_stability(int(stim_index))
    # ModelRunner.combine_results(StimuliDiff(), "model_stability.csv")


if __name__ == '__main__':
    main()
