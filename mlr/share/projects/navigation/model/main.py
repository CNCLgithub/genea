from mlr.share.projects.navigation.model.planner import NavModel
from mlr.share.projects.navigation.model.stimuli import StimuliPairs, StimuliDiff
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavAgent
from mlr.share.projects.navigation.utils.stimuli_utils import StimulusItem


class Experiment:
    @staticmethod
    def run_test_stimuli():
        stimuli_pair_dirpath = FileUtils.get_dir_list_in_directory(StimuliPairs().get_stimuli_set_dirpath())[0]

        nav_model = NavModel(NavAgent.TALOS_LEGS, stimuli_pair_dirpath)
        nav_model.add_walk_task()
        nav_model.run_dynamics()

    @staticmethod
    def run_pair_platforms(total_runs=50):
        for stimuli_pair_dirpath in FileUtils.get_dir_list_in_directory(StimuliPairs().get_stimuli_set_dirpath()):
            for run_num in range(1, total_runs + 1):
                nav_model = NavModel(NavAgent.TALOS_LEGS, stimuli_pair_dirpath)
                nav_model.add_jump_task()
                nav_model.run_dynamics()
                nav_model.save_result_to_outfile(run_num)

    @staticmethod
    def run_nav_model_on_diff():
        stim_item_dirpath_list = FileUtils.get_dir_list_in_directory(StimuliDiff().get_stimuli_set_dirpath())
        for stim_item_dirpath in stim_item_dirpath_list:
            stim_item = StimulusItem(stim_item_dirpath, StimuliDiff())

            stim_item_name = stim_item.get_stimulus_item_name()
            condition1 = int(stim_item_name.split("_")[1]) % 5 == 0 and int(stim_item_name.split("_")[1]) != 5
            condition2 = int(stim_item_name.split("_")[1]) == 1
            condition3 = int(stim_item_name.split("_")[1]) == 0
            if condition1 or condition2 or condition3:
                continue

            stim_item.load_stimuli_from_library()


if __name__ == '__main__':
    Experiment.run_test_stimuli()
