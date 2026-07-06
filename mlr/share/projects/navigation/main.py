import click

from mlr.share.projects.navigation.model.planner import NavModel
from mlr.share.projects.navigation.model.stimuli import StimuliDiff
from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.config_utils import CoreConfig
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class Experiment:
    @staticmethod
    def run_exp_diff(stim_index):
        stim_diff = StimuliDiff()
        stim_dir_list = FileUtils.get_dir_list_in_directory(stim_diff.get_stimuli_set_dirpath())
        stim_dir_list.sort(key=lambda x: int(x.split("/")[-1].split("_")[1]))

        stim_dirname = FileUtils.get_basename(stim_dir_list[stim_index])
        out_filepath = PathUtils.join(PathUtils.get_out_dirpath(), f"{stim_dirname}.csv")

        Msg.print_success("Running model on stimulus: " + stim_dirname)

        for _ in range(CoreConfig.EXP_ITERATIONS):
            nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliDiff(), stim_dirname))
            nav_model.run_planner()
            nav_model.print_possible_paths()
            nav_model.save_state_to_file(out_filepath)


@click.command()
@click.option("-si", "--stim_index", type=click.STRING, help="the index of the stimulus to run")
def main(stim_index):
    Experiment.run_exp_diff(int(stim_index))


if __name__ == '__main__':
    main()
