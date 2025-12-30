import os

from Cython import returns


class PathUtils:
    SLASH = "/"
    UNDERSCORE = "_"

    @staticmethod
    def join(*path_elements_list):
        if isinstance(path_elements_list, (list, tuple)) and len(path_elements_list) == 1:
            return path_elements_list[0]
        return os.path.join(path_elements_list[0], *path_elements_list[1:])

    @staticmethod
    def get_parent_dirpath(input_dirpath, levels_up_counter=1):
        return_dirpath = input_dirpath
        for _ in range(levels_up_counter):
            return_dirpath = os.path.dirname(return_dirpath)
        return return_dirpath

    @staticmethod
    def get_relative_path(path_to, path_from):
        return os.path.relpath(path_to, path_from)

    @staticmethod
    def get_root_dirpath():
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    @staticmethod
    def get_library_dirpath():
        return os.path.join(PathUtils.get_root_dirpath(), "library")

    @staticmethod
    def get_out_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "out")

    @staticmethod
    def get_out_nav_data_dirpath():
        return os.path.join(PathUtils.get_out_dirpath(), "nav_data")

    @staticmethod
    def get_platforms_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "platforms")

    @staticmethod
    def get_test_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "test")

    @staticmethod
    def get_stimuli_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "stimuli")

    @staticmethod
    def get_stimuli_set_dirpath(stimuli_set_name):
        return os.path.join(PathUtils.get_stimuli_dirpath(), stimuli_set_name)

    @staticmethod
    def get_misc_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "misc")
