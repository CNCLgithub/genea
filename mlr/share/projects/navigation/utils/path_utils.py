import os


class PathUtils:
    SLASH = "/"
    UNDERSCORE = "_"

    @staticmethod
    def join(*path_elements_list):
        if isinstance(path_elements_list, (list, tuple)) and len(path_elements_list) == 1:
            return path_elements_list[0]
        return os.path.join(path_elements_list[0], *path_elements_list[1:])

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
    def get_platforms_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "platforms")

    @staticmethod
    def get_test_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "test")

    @staticmethod
    def get_stimuli_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "stimuli")

    @staticmethod
    def get_misc_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "misc")

    @staticmethod
    def get_stimuli_urdf_dirpath(stimuli_dirpath):
        return os.path.join(stimuli_dirpath, "urdf")

    @staticmethod
    def get_stimuli_meshes_dirpath(stimuli_dirpath):
        return os.path.join(stimuli_dirpath, "meshes")

    @staticmethod
    def get_stimuli_pairs_dirpath(stimuli_name=None):
        if stimuli_name is None:
            return os.path.join(PathUtils.get_stimuli_dirpath(), "pairs")
        return os.path.join(PathUtils.get_stimuli_dirpath(), "pairs", stimuli_name)

    @staticmethod
    def get_stimuli_pairs_meshes_dirpath(stimuli_name):
        return PathUtils.get_stimuli_meshes_dirpath(PathUtils.get_stimuli_pairs_dirpath(stimuli_name))

    @staticmethod
    def get_stimuli_pairs_urdf_dirpath(stimuli_name):
        return PathUtils.get_stimuli_urdf_dirpath(PathUtils.get_stimuli_pairs_dirpath(stimuli_name))
