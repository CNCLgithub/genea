import os


class PathUtils:
    SLASH = "/"
    UNDERSCORE = "_"

    @staticmethod
    def create_path(*path_elements_list):
        if isinstance(path_elements_list, (list, tuple)) and len(path_elements_list) == 1:
            return path_elements_list[0]
        return os.path.join(path_elements_list[0], *path_elements_list[1:])

    @staticmethod
    def get_root_dir_path():
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    @staticmethod
    def get_library_dir_path():
        return os.path.join(PathUtils.get_root_dir_path(), "library")
