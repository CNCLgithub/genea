import os


class PathUtils:
    SLASH = "/"
    UNDERSCORE = "_"

    DOT_TXT = ".txt"
    DOT_G = ".g"
    DOT_F_G = "_f.g"

    DOT_PPM = ".ppm"
    DOT_PNG = ".png"

    @staticmethod
    def join(*paths):
        return os.path.join(paths[0], *paths[1:])

    @staticmethod
    def get_root_dirpath():
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    @staticmethod
    def get_library_dirpath():
        return os.path.join(PathUtils.get_root_dirpath(), "library")

    @staticmethod
    def get_test_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "test")

    @staticmethod
    def get_exp_data_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "exp_data")

    @staticmethod
    def get_out_human_data_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "out_human_data")

    @staticmethod
    def get_out_robot_data_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "out_robot_data")

    @staticmethod
    def get_out_vlm_data_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "out_vlm_data")

    @staticmethod
    def get_init_files_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "init_files")

    @staticmethod
    def get_intermediate_files_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "intermediate_files")

    @staticmethod
    def get_fin_files_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "fin_files")

    @staticmethod
    def get_vlm_files_dirpath():
        return os.path.join(PathUtils.get_library_dirpath(), "vlm_files")

    @staticmethod
    def get_vlm_exp_data_dirpath():
        return os.path.join(PathUtils.get_vlm_files_dirpath(), "vlm_exp_data")

    @staticmethod
    def get_risk_files_dirpath(noise_value=None):
        if noise_value is not None:
            return os.path.join(PathUtils.get_library_dirpath(), "risk_files_" + str(noise_value))
        return os.path.join(PathUtils.get_library_dirpath(), "risk_files")
