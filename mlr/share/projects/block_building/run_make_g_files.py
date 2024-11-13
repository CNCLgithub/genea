import click

from mlr.share.projects.block_building.utils.cpp_utils import CPPUtils
from mlr.share.projects.block_building.utils.path_utils import PathUtils


@click.command()
@click.option("-b", "--build_only", default=False, is_flag=True, help="only build C++ code, do not run python")
@click.option("-B", "--build_force", default=False, is_flag=True, help="build C++ code and run file")
@click.option('-e', '--exp_type', type=click.STRING, help='experiment type information')
@click.option('-t', '--exp_trial_num', type=click.STRING, help='experiment trial number information')
@click.option("-i", "--init_only", default=False, is_flag=True, help="only make the init g file")
def main(build_only, build_force, exp_type, exp_trial_num, init_only):
    if build_only:
        CPPUtils.make_make_g_filepath()
        return

    if build_force:
        CPPUtils.make_make_g_filepath()

    init_filename = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.DOT_G
    final_filename = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.DOT_F_G

    init_file_path = PathUtils.join(PathUtils.get_library_dirpath(), "init_files", init_filename)
    fin_file_path = PathUtils.join(PathUtils.get_library_dirpath(), "fin_files", final_filename)

    out_init_file_path = PathUtils.join(PathUtils.get_init_files_dirpath(), init_filename)
    out_final_file_path = PathUtils.join(PathUtils.get_fin_files_dirpath(), final_filename)

    arguments_list = [init_file_path, out_init_file_path]
    arguments_string = " ".join(arguments_list)
    CPPUtils.run_make_g_filepath(arguments_string)

    if not init_only:
        arguments_list = [fin_file_path, out_final_file_path]
        arguments_string = " ".join(arguments_list)
        CPPUtils.run_make_g_filepath(arguments_string)


if __name__ == '__main__':
    main()
