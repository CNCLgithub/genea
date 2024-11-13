import click

from mlr.share.projects.block_building.utils.cpp_utils import CPPUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


@click.command()
@click.option("-b", "--build_only", default=False, is_flag=True, help="only build C++ code, do not run python")
@click.option("-B", "--build_force", default=False, is_flag=True, help="build C++ code and run file")
@click.option('-e', '--exp_type', type=click.STRING, help='experiment type information')
@click.option('-t', '--exp_trial_num', type=click.STRING, help='experiment trial number information')
@click.option("-i", "--run_init", default=False, is_flag=True, help="view init file")
@click.option("-f", "--run_final", default=False, is_flag=True, help="view file to final file")
@click.option("-inter", "--run_intermediate", default=False, is_flag=True, help="view intermediate file")
@click.option('-s', '--source', type=click.STRING, help='<folder_num>/<intermediate_filename>')
@click.option('-p', '--g_file_path', default="", type=click.STRING, help='file path to a .g file')
def main(build_only, build_force, exp_type, exp_trial_num, run_init, run_final,
         run_intermediate, source, g_file_path):
    if build_only:
        CPPUtils.make_viewer()
        return

    if build_force:
        CPPUtils.make_viewer()

    file_path = None
    if len(g_file_path) > 1:
        file_path = g_file_path
    else:
        init_filename = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.DOT_G
        intermediate_folder_name = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.UNDERSCORE
        final_filename = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.DOT_F_G

        if run_init:
            file_path = PathUtils.join(PathUtils.get_exp_data_dirpath(), "init_files", init_filename)

        if run_intermediate:
            file_path = PathUtils.join(PathUtils.get_intermediate_files_dirpath(), intermediate_folder_name + source)

        if run_final:
            file_path = PathUtils.join(PathUtils.get_exp_data_dirpath(), "fin_files", final_filename)

    if file_path is None:
        Msg.print_error("ERROR [run_viewer]: must enter the type of file to view")

    arguments_list = [file_path]
    arguments_string = " ".join(arguments_list)

    CPPUtils.run_viewer(arguments_string)


if __name__ == '__main__':
    main()
