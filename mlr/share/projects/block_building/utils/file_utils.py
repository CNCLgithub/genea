import csv
import numpy as np
import os
import re
import shutil
import subprocess

from PIL import Image
from random import shuffle

from mlr.share.projects.block_building.utils.block_utils import BlockParams, BlockPosition, BlockQuaternion, BlockShape
from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.core_utils import NameUtils, ConfigUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class GFileUtils:

    @staticmethod
    def parse_g_file_path(g_file_path, block_names_list):
        file_data = open(g_file_path, "r").read()
        file_data = file_data.split("\n")

        body_position_data = {}
        body_rotation_data = {}

        joint_data = {}

        shape_data = {}

        def extract_joint_data(joint_line, block_name):
            return joint_line.split('joint (')[1].split(' ' + block_name)[0].lower()

        def extract_body_data(body_line):
            return body_line.split("<T ")[1].split(" >")[0].split(" ")

        def extract_shape_data(shape_line):
            return shape_line.split("type=ST_ssBox size=[")[1].split("]  color=")[0].split(" ")

        for line in file_data:
            for block_name_key in block_names_list:
                if "body " + block_name_key + " " in line:
                    body_data = extract_body_data(line)
                    body_position_data[block_name_key] = tuple(float(x) for x in body_data[:3])
                    body_rotation_data[block_name_key] = tuple(float(x) for x in body_data[3:])
                    continue

                if "joint" in line and block_name_key + ")" in line:
                    joint_data[block_name_key] = extract_joint_data(line, block_name_key)
                    continue

                if "shape" in line and block_name_key + " " in line:
                    shape_data[block_name_key] = extract_shape_data(line)
                    continue

        return body_position_data, body_rotation_data, joint_data, shape_data

    @staticmethod
    def get_block_params_from_parse_g_file_path(filepath, block_names_list):
        pos_dict, rot_dict, resting_on_dict, shape_dict = GFileUtils.parse_g_file_path(filepath, block_names_list)

        for block_name in block_names_list:
            if block_name not in pos_dict.keys():
                Msg.print_error("ERROR [get_block_params_from_parse_g_file_path]: " + block_name + " data not found!")
                assert False

        is_supporting_dict = {}
        total_blocks_supported = {}
        for key in block_names_list:
            is_supporting_dict[key] = NameUtils.NO_BLOCK_ON_TOP
            total_blocks_supported[key] = 0

        for key in block_names_list:
            resting_on_name = resting_on_dict[key]

            if resting_on_name not in is_supporting_dict:
                continue

            if not resting_on_name == NameUtils.TABLE:
                is_supporting_dict[resting_on_name.lower()] = key
                total_blocks_supported[resting_on_name.lower()] += 1

        blocks_params_dict = {}
        for block_name in block_names_list:
            pos = pos_dict[block_name]
            rot = rot_dict[block_name]
            shape = shape_dict[block_name]
            resting_on_name = resting_on_dict[block_name]
            is_supporting = is_supporting_dict[block_name]
            two_blocks_supported = total_blocks_supported[block_name] == 2
            block_params = BlockParams(BlockPosition(pos[0], pos[1], pos[2]),
                                       BlockQuaternion(rot[0], rot[1], rot[2], rot[3]),
                                       BlockShape(shape[0], shape[1], shape[2], shape[3]),
                                       resting_on_name,
                                       is_supporting,
                                       two_blocks_supported)
            blocks_params_dict[block_name] = block_params

        return blocks_params_dict

    @staticmethod
    def clean_g_file(g_file_path):
        new_contents = []
        with open(g_file_path, "r") as g_file:
            for line in g_file:
                new_joint_line = line

                if line.startswith("joint"):
                    if "Q=<" in line:
                        new_joint_line = line.split("Q=<")[0] + " }\n"
                    if "to=<T" in line:
                        to_key = " to=<T "
                        quaternion = new_joint_line.split(to_key)[1].split(" ")[3:7]
                        condition = all([ComputeUtils.is_close(float(value), 0.0, 2) for value in quaternion[1:]])

                        if condition:  # whether all are 0 (invalid quaternion) or last 3 are zeros (floating point)
                            l_start = new_joint_line.split(to_key)[0]
                            l_pos_list = " ".join(new_joint_line.split("to=<T")[1].split(" ")[0:4])
                            l_quaternion_list = "-0.99999 0.00001 0.00001 0.00001"
                            l_end = ">  }\n"
                            new_joint_line = " ".join([l_start, to_key, l_pos_list, l_quaternion_list, l_end])

                new_contents.append(new_joint_line)

        with open(g_file_path, "w") as g_file:
            for line in new_contents:
                g_file.write(line)

    @staticmethod
    def redefine_init_g_file(init_file_path, fin_file_path, block_names_list):
        if ConfigUtils.DEBUG_PLANNER:
            return

        with open(init_file_path) as init_file:
            init_file_data = init_file.read()

        init_block_params_dict = GFileUtils.get_block_params_from_parse_g_file_path(init_file_path, block_names_list)
        final_block_params_dict = GFileUtils.get_block_params_from_parse_g_file_path(fin_file_path, block_names_list)

        assignments_b, assignments_c = BlockParams.get_hungarian_algorithm_fits(init_block_params_dict,
                                                                                final_block_params_dict)

        # Rename blocks in init world file string corresponding to assignments found
        init_file_data_copy = init_file_data[:]

        # Rename for b blocks
        for i in reversed(range(len(assignments_b[0]))):
            init_file_data_copy = init_file_data_copy.replace('b' + str(assignments_b[0][i]),
                                                              'TEMP' + str(assignments_b[1][i]))

        init_file_data_copy = init_file_data_copy.replace('TEMP', 'b')

        # # Rename for c blocks
        for i in reversed(range(len(assignments_c[0]))):
            init_file_data_copy = init_file_data_copy.replace('c' + str(assignments_c[0][i]),
                                                              'TEMP' + str(assignments_c[1][i]))

        init_file_data_copy = init_file_data_copy.replace('TEMP', 'c')

        with open(init_file_path, 'w') as init_file:
            init_file.write(init_file_data_copy)

    @staticmethod
    def _generate_ppm_from_g_file(input_g_file_path, out_g_file_path, out_img_file_path="N"):
        arguments_list = [input_g_file_path, out_g_file_path, out_img_file_path, "1"]
        arguments_string = " ".join(arguments_list)
        return subprocess.check_output("./cpp/run_ppm_generator.exe " + arguments_string, shell=True)

    @staticmethod
    def _convert_ppm_to_png(ppm_file_path, png_file_path):
        if "ppm" not in os.path.basename(ppm_file_path):
            Msg.print_error("ERROR [convert_ppm_to_png]: please ensure that you pass a PPM file")
            return

        ppm_image = Image.open(ppm_file_path)
        ppm_image.save(png_file_path)

    @staticmethod
    def save_g_file_as_png(out_g_file_path, out_ppm_file_path, out_png_file_path):
        GFileUtils._generate_ppm_from_g_file(out_g_file_path, out_g_file_path, out_ppm_file_path)
        GFileUtils._convert_ppm_to_png(out_ppm_file_path, out_png_file_path)


class FileUtils:
    @staticmethod
    def is_file(filepath):
        return os.path.isfile(filepath)
    
    @staticmethod
    def is_dir(dirpath):
        return os.path.isdir(dirpath)

    @staticmethod
    def create_dir(dirpath, do_force_create=False):
        if os.path.isdir(dirpath):
            if do_force_create:
                shutil.rmtree(dirpath)
                os.makedirs(dirpath)
            return

        os.makedirs(dirpath)

    @staticmethod
    def create_out_file(out_file_path):
        FileUtils.create_dir(os.path.dirname(out_file_path))
        file = open(out_file_path, 'wb')
        file.close()

    @staticmethod
    def write_row_to_file(out_file_path, row_data):
        """
        Writes a new row into the given file
        :param out_file_path: name of the file that is to be written in
        :param row_data: the entire row entry that is to be written into the file
        :return: None
        """
        if not os.path.isfile(out_file_path):
            FileUtils.create_out_file(out_file_path)

        try:
            file = open(out_file_path, 'a')
            writer = csv.writer(file)
            writer.writerow(row_data)
            file.close()

        except Exception as e:
            Msg.print_error("ERROR [write_row_to_file]: could not write into " + out_file_path)
            Msg.print_error(str(e))
            assert False

    @staticmethod
    def copy_file(src_filepath, dest_filepath):
        shutil.copy2(src_filepath, dest_filepath)

    @staticmethod
    def get_file_basename(filepath, is_attached=True):
        if is_attached:
            return os.path.basename(filepath)
        return os.path.splitext(os.path.basename(filepath))

    @staticmethod
    def get_dir_list_in_directory(dirpath):
        try:
            return list(filter(os.path.isdir, [os.path.join(dirpath, i) for i in os.listdir(dirpath)]))
        except Exception as e:
            Msg.print_error("ERROR: issue in retrieving directory list within given directory " + dirpath)
            Msg.print_error(str(e))

    @staticmethod
    def get_files_in_directory(dirpath, is_sorted=True, is_recursive=False):
        try:
            file_paths_list = list(filter(
                os.path.isfile, [os.path.join(dirpath, i) for i in os.listdir(dirpath) if not i.startswith('.')])
            )

            if is_recursive:
                for subdir in FileUtils.get_dir_list_in_directory(dirpath):
                    file_paths_list.extend(FileUtils.get_files_in_directory(subdir, is_sorted, True))

            if not is_sorted:
                return file_paths_list
            return sorted(file_paths_list, key=lambda f: [int(n) for n in re.findall(r"\d+", f)])
        except Exception as e:
            Msg.print_error("ERROR: issue in retrieving files within directory " + dirpath)
            Msg.print_error(str(e))

    @staticmethod
    def read_csv_file(filepath, delimiter=","):
        """
        Reads and returns the data within the given file,
        or returns with an error message if no such file exists
        :param filepath    : name of the file that is to be read
        :param delimiter    : the type of the delimiter
        :return: a list of list containing all the data
        """

        try:
            file = open(filepath, 'r')
            reader = csv.reader(file, delimiter=delimiter)

            data = []
            for row in reader:
                data.append(row)
            file.close()

            return data

        except Exception:
            Msg.print_error("Error while reading from " + filepath)
            raise FileNotFoundError


class GFileGenerator:
    DEFAULT_COLOR = "[.92 .75 .52]"
    RED_COLOR = "[0.9 0.2 0.2]"
    BLUE_COLOR = "[0 0 1]"
    GREEN_COLOR = "[0 1 0]"
    YELLOW_COLOR = "[1 1 0]"
    PURPLE_COLOR = "[0.5 0 0.5]"

    CENTER_LOC = 0.0, 0.0, 0.022
    PLACE_LEFT = 0.5, -0.3, 0.022
    PLACE_RIGHT = 0.5, 0.3, 0.022

    TABLE_X_LIM_MIN = -0.3
    TABLE_X_LIM_MAX = 0.3
    TABLE_Y_LIM_MIN = -0.6
    TABLE_Y_LIM_MAX = 0.6

    MIN_DIST_BETWEEN_BLOCKS = 0.12
    LARGE_DIST_BETWEEN_BLOCKS = 0.3

    TABLE_CENTER = "TABLE_CENTER"
    TABLE_LEFT_CENTER = "TABLE_LEFT_CENTER"
    TABLE_RIGHT_CENTER = "TABLE_RIGHT_CENTER"

    @staticmethod
    def _get_generic_body_line(body_name, body_color):
        return "body " + body_name + " { type=9 size=[.05 .05 .05 .005] color=" + body_color + " contact }\n"

    @staticmethod
    def _get_generic_joint_line(body_name, body_location, body_angle, block_below='table'):
        body_location_as_string = body_location.replace(",", " ")
        return "joint (" + block_below + " " + body_name + ")    { from=<T t" + \
               body_location_as_string + " d(" + \
               body_angle + " 0 0 1)> to=<T t(0 0 .025)> type=JT_rigid }\n"

    @staticmethod
    def extract_joint_data(joint_line):
        return tuple(float(i) for i in joint_line.split('from=<T t(')[1].split(')')[0].split(" "))

    @staticmethod
    def extract_refined_joint_data(joint_line):
        return tuple(float(i) for i in joint_line.split('type=JT_rigid from=<T ')[1].split(' > ')[0].split(" "))

    @staticmethod
    def _validate_sampled_loc(sampled_loc, all_loc_list, min_dist_between_blocks):
        def _l2_distance(loc1, loc2):
            return np.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)

        for existing_loc in all_loc_list:
            if _l2_distance(existing_loc, sampled_loc) < min_dist_between_blocks:
                return False
        return True

    @staticmethod
    def _get_limits(limit, table_location):
        x_center = 0.0
        y_center = 0.0
        x_limit_min = max(-limit, GFileGenerator.TABLE_X_LIM_MIN)
        x_limit_max = min(limit, GFileGenerator.TABLE_X_LIM_MAX)
        y_limit_min = max(-limit, GFileGenerator.TABLE_Y_LIM_MIN)
        y_limit_max = min(limit, GFileGenerator.TABLE_Y_LIM_MAX)
        if table_location == GFileGenerator.TABLE_LEFT_CENTER:
            y_center = -0.25
            y_limit_min += y_center
            y_limit_max = -0.1
        if table_location == GFileGenerator.TABLE_RIGHT_CENTER:
            y_center = 0.25
            y_limit_min = 0.1
            y_limit_max += y_center
        return (x_center, y_center), x_limit_min, x_limit_max, y_limit_min, y_limit_max

    @staticmethod
    def _sample_loc_on_table(loc_list, table_loc, total_blocks_num, min_dist_between_blocks) -> tuple:
        limit = 0.001 * total_blocks_num
        center, x_limit_min, x_limit_max, y_limit_min, y_limit_max = GFileGenerator._get_limits(limit, table_loc)

        sampled_loc = [ComputeUtils.sample_trunc_normal(center[0], x_limit_min, x_limit_max),
                       ComputeUtils.sample_trunc_normal(center[1], y_limit_min, y_limit_max)]
        while not GFileGenerator._validate_sampled_loc(sampled_loc, loc_list, min_dist_between_blocks):
            sampled_loc = [ComputeUtils.sample_trunc_normal(center[0], x_limit_min, x_limit_max),
                           ComputeUtils.sample_trunc_normal(center[1], y_limit_min, y_limit_max)]
            limit += 0.001
            center, x_limit_min, x_limit_max, y_limit_min, y_limit_max = GFileGenerator._get_limits(limit, table_loc)

        sampled_loc += [0.022]
        return tuple(sampled_loc)

    @staticmethod
    def init_g_file(out_init_filename, out_fin_filename, trial_name,
                    div_block_num: list, div_color: list, div_table_loc: list):
        """
        creates a .g file with a randomly assigned initial configuration of the world
        NOTE: total number of block types must not exceed 2
        :param out_init_filename    : name of the init output file
        :param out_fin_filename     : name of the final output file
        :param trial_name           : set when there is some special initial configuration to load
        :param div_block_num        : a list representing the number of blocks in each category
        :param div_color            : a list representing the colors used for each category of blocks
        :param div_table_loc        : a list that orients each of the 'bins' to a particular side of the table
        """

        if not len(div_block_num) == len(div_color):
            Msg.print_error("ERROR [init_g_random]: bins and colors are not correctly defined")
            assert False

        trial_names_special = ["13_1", "13_2", "27_1", "27_2"]
        if trial_name in trial_names_special:
            if out_fin_filename is None:
                Msg.print_error("ERROR [init_g_random]: out filename not specified for the special trial names")
                assert False
            return GFileGenerator.init_g_13_27(out_init_filename, out_fin_filename, div_color, trial_name)

        g_file_contents = []
        final_g_file_contents = []

        trial_names_special = ["9_1", "9_2", "17_1", "17_2", "23_1", "23_2", "26_1", "31_1", "31_2", "57_1", "57_2"]

        generic_file_path = os.path.join(PathUtils.get_exp_data_dirpath(), "generic.txt")
        if trial_name in trial_names_special:
            generic_file_path = os.path.join(PathUtils.get_exp_data_dirpath(), "generic_" + trial_name + ".txt")

        with open(generic_file_path, "r") as header_file:
            for line in header_file:
                g_file_contents.append(line)
                final_g_file_contents.append(line)

        total_blocks = sum([int(i) for i in div_block_num])

        loc_avoid_list = [GFileGenerator.PLACE_LEFT, GFileGenerator.PLACE_RIGHT, GFileGenerator.CENTER_LOC]
        final_avoid_list = ["7_2", "11_1", "11_2", "15_1", "15_2", "16_2", "18_2", "19_2", "20_1", "20_2",
                            "21_1", "21_2", "22_2", "23_2", "25_1", "25_2", "26_1", "26_2", "28_1", "28_2", "29_2",
                            "30_2", "33_1", "33_2", "34_1", "34_2",
                            "51_1", "52_1", "53_1", "54_1", "55_1", "56_1", "57_1", "57_2"]
        if trial_name in final_avoid_list:
            with open(out_fin_filename, "r") as header_file:
                for line in header_file:
                    if "joint" in line and "table" in line:
                        loc_avoid_list.append(GFileGenerator.extract_refined_joint_data(line))

        for line in g_file_contents:
            if "joint" in line:
                loc_avoid_list.append(GFileGenerator.extract_joint_data(line))

        block_name_character = 'b'
        b_counter = 0
        c_counter = 0
        counter = 0

        block_names_list = []
        block_colors_list = []
        block_loc_list = [] + loc_avoid_list
        block_rot_list = []
        for div_index, num_of_blocks in enumerate(div_block_num):
            for block_counter in range(counter, num_of_blocks + counter):
                block_names_list.append(block_name_character + str(block_counter))
                block_colors_list.append(div_color[div_index])

                if trial_name == "26_1":
                    sampled_loc = GFileGenerator._sample_loc_on_table(block_loc_list, div_table_loc[div_index],
                                                                      total_blocks,
                                                                      GFileGenerator.LARGE_DIST_BETWEEN_BLOCKS)
                else:
                    sampled_loc = GFileGenerator._sample_loc_on_table(block_loc_list, div_table_loc[div_index],
                                                                      total_blocks,
                                                                      GFileGenerator.MIN_DIST_BETWEEN_BLOCKS)
                block_loc_list.append(sampled_loc)
                block_rot_list.append(np.random.uniform(0, 90, 1)[0])
            block_name_character = chr(ord(block_name_character) + 1)
            if block_name_character == "d":
                block_name_character = 'b'

            if block_name_character == "b":
                b_counter += num_of_blocks
                counter = c_counter
            else:
                c_counter += num_of_blocks
                counter = b_counter

        block_loc_list = block_loc_list[len(loc_avoid_list):]

        for block_name, block_color in zip(block_names_list, block_colors_list):
            if trial_name == "26_1":
                continue
            g_file_contents.append(GFileGenerator._get_generic_body_line(block_name, block_color))

        for block_name, block_loc, theta in zip(block_names_list, block_loc_list, block_rot_list):
            g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name, str(block_loc), str(theta)))

        with open(out_init_filename, "w") as g_file:
            for line in g_file_contents:
                g_file.write(line)

        if trial_name == "18_1":
            final_b_block_loc_list = [] + loc_avoid_list + block_loc_list
            final_c_block_loc_list = [] + loc_avoid_list + block_loc_list
            b_blocks_to_move = []
            c_blocks_to_move = []

            final_block_loc_list = []
            for block_name, block_loc in zip(block_names_list, block_loc_list):
                if block_name.startswith("b") and block_loc[1] > -0.005:
                    b_blocks_to_move.append(block_name)
                if block_name.startswith("b") and block_loc[1] <= -0.005:
                    final_b_block_loc_list.append(block_loc)

                if block_name.startswith("c") and block_loc[1] < 0.005:
                    c_blocks_to_move.append(block_name)
                if block_name.startswith("c") and block_loc[1] >= 0.005:
                    final_c_block_loc_list.append(block_loc)

            for block_name, block_loc in zip(block_names_list, block_loc_list):
                if block_name in b_blocks_to_move:
                    sampled_loc = GFileGenerator._sample_loc_on_table(final_b_block_loc_list,
                                                                      GFileGenerator.TABLE_LEFT_CENTER,
                                                                      total_blocks,
                                                                      GFileGenerator.MIN_DIST_BETWEEN_BLOCKS)
                    final_b_block_loc_list.append(sampled_loc)
                    final_block_loc_list.append(sampled_loc)
                elif block_name in c_blocks_to_move:
                    sampled_loc = GFileGenerator._sample_loc_on_table(final_c_block_loc_list,
                                                                      GFileGenerator.TABLE_RIGHT_CENTER,
                                                                      total_blocks,
                                                                      GFileGenerator.MIN_DIST_BETWEEN_BLOCKS)
                    final_c_block_loc_list.append(sampled_loc)
                    final_block_loc_list.append(sampled_loc)
                else:
                    final_block_loc_list.append(block_loc)

            for block_name, block_color in zip(block_names_list, block_colors_list):
                final_g_file_contents.append(GFileGenerator._get_generic_body_line(block_name, block_color))

            for block_name, block_loc, block_rot in zip(block_names_list, final_block_loc_list, block_rot_list):
                if block_name in b_blocks_to_move or block_name in c_blocks_to_move:
                    theta = np.random.uniform(0, 90, 1)[0]
                else:
                    theta = block_rot
                final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                    str(block_loc),
                                                                                    str(theta)))

            with open(out_fin_filename, "w") as g_file:
                for line in final_g_file_contents:
                    g_file.write(line)

    @staticmethod
    def init_g_13_27(out_init_filename, out_final_filename, div_color: list, trial_name):
        def _get_block_loc(loc_x, loc_y):
            return tuple([loc_x, loc_y, 0.022])

        init_g_file_contents = []
        final_g_file_contents = []

        generic_file_path = os.path.join(PathUtils.get_exp_data_dirpath(), "generic.txt")
        with open(generic_file_path, "r") as header_file:
            for line in header_file:
                init_g_file_contents.append(line)
                final_g_file_contents.append(line)

        minority = 'b'
        block_names_list = ['b' + str(b) for b in range(5)] + ['c' + str(c) for c in range(17)]
        if trial_name == '13_2' or trial_name == '27_2':
            minority = 'c'
            block_names_list = ['b' + str(b) for b in range(17)] + ['c' + str(c) for c in range(5)]

        shuffle(block_names_list)

        x = 0.028
        y = 0.044

        x_fin = 0
        y_fin = -0.0

        first = 0
        final_blocks_placed = 0

        stacked = []
        for block_name in block_names_list:
            if block_name[0] == 'b':
                block_color = div_color[0]
            else:
                block_color = div_color[1]

            init_g_file_contents.append(GFileGenerator._get_generic_body_line(block_name, block_color))
            final_g_file_contents.append(GFileGenerator._get_generic_body_line(block_name, block_color))

            x = -x  # alternate x
            if x < 0:
                y = y + 0.055  # increment y every other alternation

            # Place body joint
            init_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                               str(_get_block_loc(x, y)),
                                                                               str(0)))

            if trial_name == '13_2':
                if block_name[0] == minority:
                    final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                        str(_get_block_loc(x_fin,
                                                                                                           y_fin)),
                                                                                        str(0)))
                    y_fin -= 0.055
                else:
                    final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                        str(_get_block_loc(x, y)),
                                                                                        str(0)))
            elif trial_name == '27_2':
                if block_name[0] == minority and final_blocks_placed < 3:
                    if final_blocks_placed == 0:
                        first = block_name
                    if final_blocks_placed == 2:
                        final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                            str(tuple([x_fin,
                                                                                                       -0.025, 0.026])),
                                                                                            str(0), first))
                    else:
                        final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                            str(_get_block_loc(x_fin,
                                                                                                               y_fin)),
                                                                                            str(0)))
                        y_fin -= 0.055
                    final_blocks_placed += 1
                    stacked.append(block_name)
                else:
                    final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                        str(_get_block_loc(x, y)),
                                                                                        str(0)))
            elif trial_name == '13_1':
                if not block_name[0] == minority and final_blocks_placed < 5:
                    final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                        str(_get_block_loc(x_fin,
                                                                                                           y_fin)),
                                                                                        str(0)))
                    y_fin -= 0.055
                    final_blocks_placed += 1
                else:
                    final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                        str(_get_block_loc(x, y)),
                                                                                        str(0)))
            elif trial_name == '27_1':
                if not block_name[0] == minority and final_blocks_placed < 3:
                    if final_blocks_placed == 0:
                        first = block_name
                    if final_blocks_placed == 2:
                        final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                            str(tuple([x_fin,
                                                                                                       -0.025, 0.026])),
                                                                                            str(0), first))
                    else:
                        final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                            str(_get_block_loc(x_fin,
                                                                                                               y_fin)),
                                                                                            str(0)))
                        y_fin -= 0.055
                    final_blocks_placed += 1
                    stacked.append(block_name)
                else:
                    final_g_file_contents.append(GFileGenerator._get_generic_joint_line(block_name,
                                                                                        str(_get_block_loc(x, y)),
                                                                                        str(0)))

        with open(out_init_filename, "w") as g_file:
            for line in init_g_file_contents:
                g_file.write(line)

        with open(out_final_filename, "w") as g_file:
            for line in final_g_file_contents:
                g_file.write(line)

    @staticmethod
    def generate_random_final_file(final_file_path, out_file_path, block_names_list, mean, bound, sigma, seed):
        np.random.seed(seed)

        def _get_random_truncated_normal():
            random_value = ComputeUtils.sample_trunc_normal(mean, bound[0], bound[1], sigma)
            return round(random_value, 5)

        def _get_random_sign():
            return np.random.randint(0, 2) * 2 - 1

        def _add_noise_to_joint_data(joint_line):
            split1 = 'type=JT_rigid from=<T '
            split2 = ' > to'
            part1 = joint_line.split(split1)[0]
            part2 = joint_line.split(split1)[1].split(split2)[0].split(" ")
            part3 = joint_line.split(split1)[1].split(split2)[1]

            part2[0] = str(float(part2[0]) + _get_random_sign() * _get_random_truncated_normal())
            part2[1] = str(float(part2[1]) + _get_random_sign() * _get_random_truncated_normal())

            return "".join([part1, split1, " ".join(part2), split2, part3])

        def _get_block_names_from_joint(joint_line):
            split1 = 'joint ('
            split2 = '){'
            return joint_line.split(split1)[1].split(split2)[0].split(" ")

        final_g_file_contents = []

        file_data = open(final_file_path, "r").read()
        file_data = file_data.split("\n")

        for line in file_data:
            if "joint" in line and "table" not in line:
                block_name1, block_name2 = _get_block_names_from_joint(line)
                if block_name1 in block_names_list and block_name2 in block_names_list:
                    final_g_file_contents.append(_add_noise_to_joint_data(line))
            else:
                final_g_file_contents.append(line)

        with open(out_file_path, "w") as g_file:
            for line in final_g_file_contents:
                g_file.write(line + "\n")


if __name__ == '__main__':
    test_file_path = os.path.join(PathUtils.get_test_dirpath(), "test.g")
