from mlr.share.projects.navigation.model.planner import NavModel
from mlr.share.projects.navigation.model.stimuli import StimuliDiff
from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class Experiment:
    @staticmethod
    def run_exp_diff():
        stim_diff = StimuliDiff()
        for stimulus_dirname in FileUtils.get_dir_list_in_directory(stim_diff.get_stimuli_set_dirpath()):
            Msg.print_success("Running model on stimulus: " + stimulus_dirname)
            nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliDiff(), stimulus_dirname))
            nav_model.run_planner()
            nav_model.get_all_valid_paths()


if __name__ == '__main__':
    Experiment.run_exp_diff()
