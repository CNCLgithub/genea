from mlr.share.projects.block_building.model.exp import ExpType
from mlr.share.projects.block_building.model.vlm.planner import OpenAIPlanner
from mlr.share.projects.block_building.utils.file_utils import FileUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class VLMExperiment:
    def __init__(self, exp_type, exp_trials_list):
        self._exp_type = exp_type
        self._exp_trials_list = exp_trials_list

    def _get_img_filenames(self):
        img_filepaths_list = []

        root_dirpath = PathUtils.get_vlm_files_dirpath()
        if self._exp_type == ExpType.DIFFICULTY:
            img_dirpath = PathUtils.join(root_dirpath, "diff_img_files")
        elif self._exp_type == ExpType.ACTION:
            img_dirpath = PathUtils.join(root_dirpath, "action_img_files")
        elif self._exp_type == ExpType.GOAL:
            img_dirpath = PathUtils.join(root_dirpath, "goal_img_files")
        elif self._exp_type == ExpType.HAND:
            img_dirpath = PathUtils.join(root_dirpath, "hands_img_files")
        else:
            Msg.print_error("ERROR[VLMExperiment]: Invalid experiment type: " + self._exp_type)
            assert False

        for filepath in FileUtils.get_files_in_directory(img_dirpath):
            if not filepath.endswith(".jpg") or filepath.endswith("_opt.jpg"):
                continue
            if self._exp_type == ExpType.ACTION or self._exp_type == ExpType.GOAL:
                img_filepaths_list.append([filepath, filepath.replace(".jpg", "_opt.jpg")])
            else:
                img_filepaths_list.append([filepath])

        return img_filepaths_list

    def _start_experiment(self, exp_details_filepath, out_csv_filepath):
        experiment = OpenAIPlanner(exp_details_filepath, out_csv_filepath, self._get_img_filenames())
        experiment.run_experiment()

    def _run_diff_diff_experiment(self):
        exp_details_filepath = PathUtils.join(PathUtils.get_vlm_exp_data_dirpath(), "exp_diff_difficulty.json")
        out_csv_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_diff_difficulty.csv")
        self._start_experiment(exp_details_filepath, out_csv_filepath)

    def _run_diff_est_experiment(self):
        exp_details_filepath = PathUtils.join(PathUtils.get_vlm_exp_data_dirpath(), "exp_diff_estimation.json")
        out_csv_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_diff_estimation.csv")
        self._start_experiment(exp_details_filepath, out_csv_filepath)

    def _run_action_experiment(self):
        exp_details_filepath = PathUtils.join(PathUtils.get_vlm_exp_data_dirpath(), "exp_action.json")
        out_csv_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_action.csv")
        self._start_experiment(exp_details_filepath, out_csv_filepath)

    def _run_goal_experiment(self):
        exp_details_filepath = PathUtils.join(PathUtils.get_vlm_exp_data_dirpath(), "exp_goal.json")
        out_csv_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_goal.csv")
        self._start_experiment(exp_details_filepath, out_csv_filepath)

    def _run_hands_experiment(self):
        exp_details_filepath = PathUtils.join(PathUtils.get_vlm_exp_data_dirpath(), "exp_hands.json")
        out_csv_filepath = PathUtils.join(PathUtils.get_out_vlm_data_dirpath(), "out_hands.csv")
        self._start_experiment(exp_details_filepath, out_csv_filepath)

    def run_experiment(self):
        if self._exp_type == ExpType.DIFFICULTY:
            self._run_diff_diff_experiment()
            self._run_diff_est_experiment()
        elif self._exp_type == ExpType.ACTION:
            self._run_action_experiment()
        elif self._exp_type == ExpType.GOAL:
            self._run_goal_experiment()
        elif self._exp_type == ExpType.HAND:
            self._run_hands_experiment()
        else:
            Msg.print_error("ERROR[VLMExperiment]: Invalid experiment type: " + self._exp_type)
            assert False
