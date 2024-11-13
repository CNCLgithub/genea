import click

from mlr.share.projects.block_building.model.experiment import Experiment
from mlr.share.projects.block_building.utils.core_utils import ConfigUtils
from mlr.share.projects.block_building.utils.cpp_utils import CPPUtils
from mlr.share.projects.block_building.utils.file_utils import FileUtils, GFileGenerator
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


def run_risk_test(exp_trial_num, exp_type, num_iter, random_seed):
    core_id = exp_type + PathUtils.UNDERSCORE + exp_trial_num

    core_dir_path = PathUtils.join(PathUtils.get_risk_files_dirpath(), core_id)
    FileUtils.create_dir(core_dir_path)

    final_g_file_path = PathUtils.join(PathUtils.get_fin_files_dirpath(), core_id + PathUtils.DOT_F_G)

    block_names_list = "#".join(Experiment.get_block_names(exp_type=exp_type, exp_trial_num=exp_trial_num))

    for i in range(1, int(num_iter) + 1):
        Msg.print_success("--> " + core_id + " " + str(i))

        out_g_filename = "risk_" + str(i) + PathUtils.DOT_G
        out_txt_filename = "risk_" + str(i) + PathUtils.DOT_TXT

        out_g_file_path = PathUtils.join(PathUtils.get_risk_files_dirpath(), core_id, out_g_filename)
        out_risk_value_file_path = PathUtils.join(PathUtils.get_risk_files_dirpath(), core_id, out_txt_filename)

        GFileGenerator.generate_random_final_file(final_g_file_path, out_g_file_path, block_names_list, 0.0,
                                                  (-ConfigUtils.NOISE_GAUSSIAN_STD, ConfigUtils.NOISE_GAUSSIAN_STD),
                                                  ConfigUtils.NOISE_GAUSSIAN_STD,
                                                  random_seed + 10 * i)

        arguments_list = [out_g_file_path, block_names_list, out_risk_value_file_path]
        arguments_string = " ".join(arguments_list)
        CPPUtils.run_risk_test(arguments_string)


@click.command()
@click.option("-b", "--build_only", default=False, is_flag=True, help="only build C++ code, do not run python")
@click.option("-B", "--build_force", default=False, is_flag=True, help="build C++ code and run file")
@click.option('-e', '--exp_type', type=click.STRING, help='experiment type information')
@click.option('-t', '--exp_trial_num', type=click.STRING, help='experiment trial number information')
@click.option('-n', '--num_iter', type=click.STRING, help='the number of iterations to run')
def main(build_only, build_force, exp_type, exp_trial_num, num_iter):
    if build_only:
        CPPUtils.make_risk_test()
        return

    if build_force:
        CPPUtils.make_risk_test()

    run_risk_test(exp_trial_num, exp_type, num_iter, 0)


if __name__ == '__main__':
    main()
