import click
import multiprocessing
import numpy as np
import time

from mlr.share.projects.block_building.utils.file_utils import FileUtils, GFileUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


def make_pmm_png_image(g_file_path):
    g_dir_path = FileUtils.is_dir(g_file_path)
    g_file_basename = FileUtils.get_file_basename(g_file_path).split(".")[0]

    ppm_file_path = PathUtils.join(g_dir_path, "ppm_files", g_file_basename + PathUtils.DOT_PPM)
    png_file_path = PathUtils.join(g_dir_path, "png_files", g_file_basename + PathUtils.DOT_PNG)

    GFileUtils.save_g_file_as_png(g_file_path, ppm_file_path, png_file_path)


@click.command()
@click.option('-p', '--g_files_dir', default="", type=click.STRING, help='file path to a .g file')
def main(g_files_dir):
    if not FileUtils.is_dir(g_files_dir):
        Msg.print_error("ERROR [run_image_generator]: must enter the dir where g files are stored")
        assert False

    ppm_dir_path = PathUtils.join(g_files_dir, "ppm_files")
    FileUtils.create_dir(ppm_dir_path)

    png_dir_path = PathUtils.join(g_files_dir, "png_files")
    FileUtils.create_dir(png_dir_path)

    file_paths_in_dir = FileUtils.get_files_in_directory(g_files_dir)

    g_file_paths_list = [fp for fp in file_paths_in_dir if FileUtils.get_file_basename(fp).endswith(".g")]

    cores = multiprocessing.cpu_count()
    pool = multiprocessing.Pool(processes=cores, initializer=np.random.seed(int(time.time())))
    pool.map(make_pmm_png_image, g_file_paths_list)
    pool.close()
    pool.terminate()
    pool.join()


if __name__ == '__main__':
    main()
