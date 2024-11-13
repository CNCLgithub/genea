import os

from mlr.share.projects.block_building.utils.path_utils import PathUtils


class CPPUtils:
    @staticmethod
    def _get_exe_dirpath():
        return "cpp"

    @staticmethod
    def _run_command(exe_filename, arguments, run_gdb=False):
        exe_path = PathUtils.join(CPPUtils._get_exe_dirpath(), exe_filename)
        
        args = ""
        if len(arguments) > 1:
            args = " " + arguments
        
        if run_gdb:
            return os.system("gdb --args ./" + exe_path + args)
        return os.system("./" + exe_path + args)

    @staticmethod
    def _make_command(make_filename):
        return os.system("make --always-make --silent -f " + make_filename)

    @staticmethod
    def run_physx(arguments="", run_gdb=False):
        return CPPUtils._run_command("run_physx.exe", arguments, run_gdb)

    @staticmethod
    def run_make_g_filepath(arguments="", run_gdb=False):
        return CPPUtils._run_command("make_g_file.exe", arguments, run_gdb)

    @staticmethod
    def run_risk_test(arguments="", run_gdb=False):
        return CPPUtils._run_command("run_risk_test.exe", arguments, run_gdb)

    @staticmethod
    def run_stability(arguments="", run_gdb=False):
        return CPPUtils._run_command("run_stability.exe", arguments, run_gdb)

    @staticmethod
    def run_viewer(arguments="", run_gdb=False):
        return CPPUtils._run_command("run_viewer.exe", arguments, run_gdb)

    @staticmethod
    def make_physx():
        return CPPUtils._make_command("makefile_run_viewer")

    @staticmethod
    def make_make_g_filepath():
        return CPPUtils._make_command("makefile_run_make_g_file")

    @staticmethod
    def make_risk_test():
        return CPPUtils._make_command("makefile_run_risk_test")

    @staticmethod
    def make_stability():
        return CPPUtils._make_command("makefile_run_stability")

    @staticmethod
    def make_viewer():
        return CPPUtils._make_command("makefile_run_viewer")
