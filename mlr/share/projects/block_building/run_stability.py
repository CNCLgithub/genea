import click

from mlr.share.projects.block_building.model.experiment import Experiment
from mlr.share.projects.block_building.utils.cpp_utils import CPPUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


@click.command()
@click.option("-b", "--build_only", default=False, is_flag=True, help="only build C++ code, do not run python")
@click.option("-B", "--build_force", default=False, is_flag=True, help="build C++ code and run file")
@click.option("-i", "--run_init", default=False, is_flag=True, help="init file to init file")
@click.option("-f", "--run_final", default=False, is_flag=True, help="final file to final file")
@click.option("-i2f", "--run_init_to_final", default=False, is_flag=True, help="final file to final file")
@click.option("-inter", "--run_intermediate", default=False, is_flag=True, help="intermediate file to intermediate file")
@click.option('-s', '--source_dir', type=click.STRING, help='<folder_num>/<intermediate_filename>')
@click.option('-e', '--exp_type', type=click.STRING, help='experiment type information')
@click.option('-t', '--exp_trial_num', type=click.STRING, help='experiment trial number information')
@click.option('-n', '--no_save', default=False, is_flag=True, help='do not save resulting state')
@click.option('-p', '--g_file_path', default="", type=click.STRING, help='file path to a .g file')
def main(build_only, build_force,
         run_init, run_final, run_init_to_final, run_intermediate, source_dir,
         exp_type, exp_trial_num, no_save, g_file_path):
    if build_only:
        CPPUtils.make_stability()
        return

    if build_force:
        CPPUtils.make_stability()

    init_filename = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.DOT_G
    intermediate_folder_name = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.UNDERSCORE
    final_filename = exp_type + PathUtils.UNDERSCORE + exp_trial_num + PathUtils.DOT_F_G

    filepath = None
    if len(g_file_path) > 1:
        filepath = g_file_path
    else:
        if run_init:
            filepath = PathUtils.join(PathUtils.get_exp_data_dirpath(), "init_files", init_filename)
        elif run_intermediate:
            filepath = PathUtils.join(PathUtils.get_intermediate_files_dirpath(), intermediate_folder_name + source_dir)
        elif run_final:
            filepath = PathUtils.join(PathUtils.get_exp_data_dirpath(), "fin_files", final_filename)

    if filepath is None:
        Msg.print_error("ERROR [run_viewer]: must enter the type of file to view")
        assert False

    out_file_path = PathUtils.join(PathUtils.get_init_files_dirpath(), init_filename)
    if run_final or run_init_to_final:
        out_file_path = PathUtils.join(PathUtils.get_fin_files_dirpath(), final_filename)

    block_names_list = "#".join(Experiment.get_block_names(exp_type=exp_type, exp_trial_num=exp_trial_num))

    if no_save:
        out_file_path = "0"

    arguments_list = [filepath, block_names_list, out_file_path, "1"]
    arguments_string = " ".join(arguments_list)
    CPPUtils.run_stability(arguments_string)


if __name__ == '__main__':
    main()
