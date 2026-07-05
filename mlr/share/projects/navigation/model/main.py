from mlr.share.projects.navigation.model.planner import NavModel
from mlr.share.projects.navigation.model.stimuli import StimuliDiff
from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class Experiment:
    @staticmethod
    def run_exp_diff():
        out_filepath = PathUtils.get_out_dirpath() + "exp_v2.csv"

        stim_diff = StimuliDiff()
        stim_dir_list = FileUtils.get_dir_list_in_directory(stim_diff.get_stimuli_set_dirpath())
        stim_dir_list.sort(key=lambda x: int(x.split("/")[-1].split("_")[1]))

        for stimulus_dirname in stim_dir_list:
            Msg.print_success("Running model on stimulus: " + stimulus_dirname)

            for _ in range(10):
                nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliDiff(), stimulus_dirname))
                nav_model.run_planner()
                nav_model.print_possible_paths()
                nav_model.save_state_to_file(out_filepath)


if __name__ == '__main__':
    Experiment.run_exp_diff()
