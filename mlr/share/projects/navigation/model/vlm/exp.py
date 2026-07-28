from mlr.share.projects.navigation.model.exp_type import ExpType
from mlr.share.projects.navigation.model.vlm.planner import OpenAIPlanner
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.path_utils import PathUtils


class VLMExperiment:
    def __init__(self, exp_type):
        self._exp_type = exp_type

    def _get_img_filenames(self):
        img_filepaths_list = []

        root_dirpath = PathUtils.get_vlm_img_files_dirpath()
        if self._exp_type == ExpType.NAV_DIFFICULTY:
            img_dirpath = PathUtils.join(root_dirpath, "diff_img_files")
        else:
            Msg.print_error("ERROR[VLMExperiment]: Invalid experiment type: " + self._exp_type)
            assert False

        for filepath in FileUtils.get_files_in_directory(img_dirpath):
            if not filepath.endswith(".jpg"):
                continue
            img_filepaths_list.append([filepath])

        return img_filepaths_list

    def _start_experiment(self, exp_details_filepath, out_csv_filepath):
        experiment = OpenAIPlanner(exp_details_filepath, out_csv_filepath, self._get_img_filenames())
        experiment.run_experiment()

    def _run_nav_diff_experiment(self):
        exp_details_filepath = PathUtils.join(PathUtils.get_vlm_json_files_dirpath(), "exp_nav_difficulty.json")
        out_csv_filepath = PathUtils.join(PathUtils.get_out_vlm_dirpath(), "out_nav_difficulty.csv")
        self._start_experiment(exp_details_filepath, out_csv_filepath)

    def run_experiment(self):
        if self._exp_type == ExpType.NAV_DIFFICULTY:
            self._run_nav_diff_experiment()
        else:
            Msg.print_error("ERROR[VLMExperiment]: Invalid experiment type: " + self._exp_type)
            assert False
