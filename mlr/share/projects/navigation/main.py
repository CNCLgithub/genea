from concurrent.futures import ProcessPoolExecutor, as_completed

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
    def run_stimulus(stimulus_dirname, out_filepath):
        Msg.print_success("Running model on stimulus: " + stimulus_dirname)

        for _ in range(CoreConfig.EXP_ITERATIONS):
            nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliDiff(), stimulus_dirname))
            nav_model.run_planner()
            nav_model.print_possible_paths()
            nav_model.save_state_to_file(out_filepath)

    @staticmethod
    def run_exp_diff():
        stim_diff = StimuliDiff()
        stim_dir_list = FileUtils.get_dir_list_in_directory(stim_diff.get_stimuli_set_dirpath())
        stim_dir_list.sort(key=lambda x: int(x.split("/")[-1].split("_")[1]))

        with ProcessPoolExecutor(max_workers=CoreConfig.EXP_WORKERS) as executor:
            process_result_list = []

            for stimulus_dirpath in stim_dir_list:
                stimulus_dirname = FileUtils.get_basename(stimulus_dirpath)
                out_filepath = PathUtils.join(PathUtils.get_out_dirpath(), f"{stimulus_dirname}.csv")
                process_result_list.append(executor.submit(Experiment.run_stimulus, stimulus_dirpath, out_filepath))

            for processed_result in as_completed(process_result_list):
                processed_result.result()


if __name__ == '__main__':
    Experiment.run_exp_diff()
