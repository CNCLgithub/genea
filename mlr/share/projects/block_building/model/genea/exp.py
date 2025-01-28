from mlr.share.projects.block_building.model.exp import ExpType
from mlr.share.projects.block_building.model.genea.blocks import Block
from mlr.share.projects.block_building.model.genea.planner.search_tree import WorldStateTree
from mlr.share.projects.block_building.utils.core_utils import NameUtils, ConfigUtils
from mlr.share.projects.block_building.utils.cpp_utils import CPPUtils
from mlr.share.projects.block_building.utils.file_utils import GFileUtils, FileUtils, GFileGenerator
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class GeneaExperiment:
    def __init__(self,
                 exp_type,
                 exp_trial_num,
                 exp_trial_move_num,
                 folder_counter,
                 max_steps_per_plan):

        self.exp_type = ExpType.get_exp_type(exp_type)
        self.exp_trial_num = exp_trial_num
        self.folder_counter = folder_counter

        self.exp_name_to_print = self.exp_type + " " + str(self.exp_trial_num)
        if ExpType.ACTION in self.exp_type:
            self.exp_name_to_print = self.exp_type + " " + str(self.exp_trial_num) + str(exp_trial_move_num)

        self.max_steps_per_plan = max_steps_per_plan

        self._init_blocks_list = None
        self._block_names_list = None

    @staticmethod
    def get_block_names(exp_type, exp_trial_num=None):
        if exp_type == ExpType.DIFFICULTY and exp_trial_num is None:
            Msg.print_error("ERROR [get_block_names]: must provide trial number for difficulty exp")
            assert False

        if exp_type == ExpType.ACTION or exp_type == ExpType.GOAL:
            return NameUtils.BLOCK_NAMES_FIVE
        elif exp_type == ExpType.HAND:
            if exp_trial_num == "4":
                return NameUtils.BLOCK_NAMES_FOUR
            elif exp_trial_num == "17":
                return NameUtils.BLOCK_NAMES_FOUR
            return NameUtils.BLOCK_NAMES_THREE
        elif exp_type == ExpType.DIFFICULTY:
            total_b_blocks = 0
            total_c_blocks = 0
            if exp_trial_num == "7_1":
                total_b_blocks = 3
            elif exp_trial_num == "7_2":
                total_b_blocks = 10
            elif exp_trial_num == "8_1":
                total_b_blocks = 5
            elif exp_trial_num == "8_2":
                total_b_blocks = 15
            elif exp_trial_num == "9_1":
                total_b_blocks = 10
            elif exp_trial_num == "9_2":
                total_b_blocks = 10
            elif exp_trial_num == "11_1":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_num == "11_2":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_num == "12_1":
                total_b_blocks = 2
            elif exp_trial_num == "12_2":
                total_b_blocks = 10
            elif exp_trial_num == "13_1":
                total_b_blocks = 5
                total_c_blocks = 17
            elif exp_trial_num == "13_2":
                total_b_blocks = 17
                total_c_blocks = 5
            elif exp_trial_num == "14_1":
                total_b_blocks = 10
            elif exp_trial_num == "14_2":
                total_b_blocks = 10
            elif exp_trial_num == "15_1":
                total_b_blocks = 5
            elif exp_trial_num == "15_2":
                total_b_blocks = 13
            elif exp_trial_num == "16_1":
                total_b_blocks = 5
            elif exp_trial_num == "16_2":
                total_b_blocks = 15
            elif exp_trial_num == "17_1":
                total_b_blocks = 2
                total_c_blocks = 8
            elif exp_trial_num == "17_2":
                total_b_blocks = 8
                total_c_blocks = 2
            elif exp_trial_num == "18_1":
                total_b_blocks = 8
                total_c_blocks = 7
            elif exp_trial_num == "18_2":
                total_b_blocks = 8
                total_c_blocks = 7
            elif exp_trial_num == "19_1":
                total_b_blocks = 10
            elif exp_trial_num == "19_2":
                total_b_blocks = 5
                total_c_blocks = 5
            elif exp_trial_num == "20_1":
                total_b_blocks = 21
            elif exp_trial_num == "20_2":
                total_b_blocks = 21
            elif exp_trial_num == "21_1":
                total_b_blocks = 4
            elif exp_trial_num == "21_2":
                total_b_blocks = 14
            elif exp_trial_num == "22_1":
                total_b_blocks = 3
            elif exp_trial_num == "22_2":
                total_b_blocks = 15
            elif exp_trial_num == "23_1":
                total_b_blocks = 6
            elif exp_trial_num == "23_2":
                total_b_blocks = 10
            elif exp_trial_num == "25_1":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_num == "25_2":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_num == "26_1":
                total_b_blocks = 4
            elif exp_trial_num == "26_2":
                total_b_blocks = 16
            elif exp_trial_num == "27_1":
                total_b_blocks = 5
                total_c_blocks = 17
            elif exp_trial_num == "27_2":
                total_b_blocks = 17
                total_c_blocks = 5
            elif exp_trial_num == "28_1":
                total_b_blocks = 12
            elif exp_trial_num == "28_2":
                total_b_blocks = 12
            elif exp_trial_num == "29_1":
                total_b_blocks = 3
            elif exp_trial_num == "29_2":
                total_b_blocks = 15
            elif exp_trial_num == "30_1":
                total_b_blocks = 3
            elif exp_trial_num == "30_2":
                total_b_blocks = 15
            elif exp_trial_num == "31_1":
                total_b_blocks = 2
                total_c_blocks = 8
            elif exp_trial_num == "31_2":
                total_b_blocks = 10
            elif exp_trial_num == "33_1":
                total_b_blocks = 12
            elif exp_trial_num == "33_2":
                total_b_blocks = 6
                total_c_blocks = 6
            elif exp_trial_num == "34_1":
                total_b_blocks = 17
            elif exp_trial_num == "34_2":
                total_b_blocks = 17
            elif exp_trial_num == "51_1":
                total_b_blocks = 5
            elif exp_trial_num == "52_1":
                total_b_blocks = 10
            elif exp_trial_num == "53_1":
                total_b_blocks = 9
            elif exp_trial_num == "54_1":
                total_b_blocks = 5
            elif exp_trial_num == "55_1":
                total_b_blocks = 10
            elif exp_trial_num == "56_1":
                total_b_blocks = 10
            elif exp_trial_num == "57_1":
                total_b_blocks = 3
                total_c_blocks = 7
            elif exp_trial_num == "57_2":
                total_b_blocks = 8
                total_c_blocks = 2

            block_names = []
            if total_b_blocks > 0:
                block_names.extend([NameUtils.NAME_DIFF_B + str(num) for num in range(total_b_blocks)])
            if total_c_blocks > 0:
                block_names.extend([NameUtils.NAME_DIFF_C + str(num) for num in range(total_c_blocks)])
            return block_names
        return None

    def _get_init_file_path(self):
        filename = self.exp_type + PathUtils.UNDERSCORE + self.exp_trial_num
        return PathUtils.join(PathUtils.get_init_files_dirpath(), filename + PathUtils.DOT_G)

    def _get_fin_file_path(self):
        filename = self.exp_type + PathUtils.UNDERSCORE + self.exp_trial_num
        return PathUtils.join(PathUtils.get_fin_files_dirpath(), filename + PathUtils.DOT_F_G)

    def get_folder_counter_value(self):
        return self.folder_counter

    def _init_block_names_list(self):
        self._block_names_list = GeneaExperiment.get_block_names(self.exp_type, self.exp_trial_num)

    def _load_blocks_from_g_files(self):
        if self._block_names_list is None:
            Msg.print_error("ERROR [_load_blocks_from_g_files]: block names were not initialized")
            assert False

        init_file_path = self._get_init_file_path()
        final_file_path = self._get_fin_file_path()

        if self.exp_type == ExpType.DIFFICULTY:
            GFileUtils.redefine_init_g_file(init_file_path, final_file_path, self._block_names_list)

        init_block_params_dict = GFileUtils.get_block_params_from_parse_g_file_path(init_file_path,
                                                                                    self._block_names_list)
        final_block_params_dict = GFileUtils.get_block_params_from_parse_g_file_path(final_file_path,
                                                                                     self._block_names_list)

        blocks_list = []
        for block_name in self._block_names_list:
            init_block_params = init_block_params_dict[block_name]
            fin_block_params = final_block_params_dict[block_name]

            blocks_list.append(Block(block_name, init_block_params, fin_block_params,
                                     self.exp_type == ExpType.DIFFICULTY))

        return blocks_list

    def _initialize_experiment(self):
        self._init_block_names_list()

        if self.exp_type == ExpType.DIFFICULTY:
            self._run_g_file_generator()

        blocks_list = self._load_blocks_from_g_files()
        if not len(blocks_list) == len(self._block_names_list):
            Msg.print_error("ERROR [_init_experiment]: blocks list passed does not contain expected number of blocks")
            assert False

        self._init_blocks_list = blocks_list

    @staticmethod
    def _remove_duplicates(zipped_input_list):
        def _extract_unique_elements_in_list(zipped_in_list):
            previous_element = object()
            for element in zipped_in_list:
                if element[0] == previous_element:
                    continue
                yield element
                previous_element = element[0]
        return list(_extract_unique_elements_in_list(sorted(zipped_input_list, key=lambda x: x[0], reverse=True)))

    @staticmethod
    def _remove_second_move_grab(zipped_input_list):
        return_list = []
        for element in zipped_input_list:
            symbolic_plan = element[0]
            second_action_detailed = symbolic_plan[2]
            second_action = [action_str[:1] for action_str in second_action_detailed.split(", ")]
            if "G" in second_action:
                continue
            return_list.append(element)
        return return_list

    def _print_planner_output(self, planner_output):
        if planner_output is None:
            Msg.print_warn("WARN []: nothing to run for " + str(self.exp_type) + "_" + str(self.exp_trial_num))
            return

        for index in range(len(planner_output)):
            plan_moves = planner_output[index][0][1:]  # skip start node
            plan_tree_paths = planner_output[index][1]
            plan_ke = planner_output[index][2][1:]  # skip start node
            plan_stability = planner_output[index][3][1:]  # skip start node

            plan_moves_to_print = "[[" + "], [".join(plan_moves) + "]]"
            plan_tree_paths_to_print = "_".join(plan_tree_paths[1:])

            start_index = 1  # skip first since it is the default if not difficulty
            if self.exp_type == ExpType.DIFFICULTY:
                start_index = 0

            plan_moves_list = plan_moves[start_index:]
            plan_kes_list = plan_ke[start_index:]
            plan_stability_list = plan_stability[start_index:]
            plan_cost_to_print = str(len(plan_moves_list))

            plan_ke_to_print = 0.0
            plan_stability_to_print = 0.0
            for move, ke, stability in zip(plan_moves_list, plan_kes_list, plan_stability_list):
                if [i[0] for i in move.split(", ")].count('P') > 0:
                    plan_ke_to_print += ke + ConfigUtils.PLACE_PENALTY
                else:
                    plan_ke_to_print += ke

                plan_stability_to_print += stability
            out_row = [self.exp_name_to_print, plan_tree_paths[0], plan_tree_paths_to_print,
                       plan_moves_to_print,
                       plan_cost_to_print,
                       str(plan_ke_to_print),
                       str(plan_stability_to_print)]

            out_file_path = PathUtils.join(PathUtils.get_out_robot_data_dirpath(), "overall_out.csv")
            FileUtils.write_row_to_file(out_file_path, out_row)

        return planner_output

    def _plan_moves(self, is_one_hand_only_plan, first_block_names_to_grab):
        Msg.print_success("-> " + str(self.exp_type) + "_" + str(self.exp_trial_num) + " " + str(self.folder_counter))

        self._initialize_experiment()

        planner = GeneaPlanner(exp_type=self.exp_type,
                               exp_trial_num=self.exp_trial_num,
                               is_diff=self.exp_type == ExpType.DIFFICULTY,
                               init_blocks_list=self._init_blocks_list,
                               max_steps_per_plan=self.max_steps_per_plan,
                               folder_counter=self.folder_counter,
                               is_one_hand_only_plan=is_one_hand_only_plan)
        planner.run_planner(first_block_name_to_grab=first_block_names_to_grab)

        returned_moves = planner.get_all_valid_moves()
        returned_tree_paths = planner.get_all_valid_tree_paths()
        returned_ke = planner.get_ke_per_valid_move()
        returned_stability = planner.get_stability_per_valid_move()

        return self._remove_duplicates(zip(returned_moves, returned_tree_paths, returned_ke, returned_stability))

    def _plan_one_hand_moves(self, first_block_names_to_grab):
        return self._plan_moves(True, first_block_names_to_grab)

    def _plan_two_hand_moves(self, first_block_names_to_grab):
        return self._plan_moves(False, first_block_names_to_grab)

    def _plan_all_possible_moves(self, first_block_names_to_grab):
        all_plans = []
        one_hand_moves = self._plan_one_hand_moves(first_block_names_to_grab)
        all_plans.extend(one_hand_moves)

        if self.exp_type == ExpType.DIFFICULTY:
            return self._remove_duplicates(all_plans)

        if self.exp_type == ExpType.HAND and len(one_hand_moves) == 0:
            all_plans.append((['START'], ['1'], [0.0], [0.0]))

        self.folder_counter += 1

        two_hand_moves = self._plan_two_hand_moves(first_block_names_to_grab)
        all_plans.extend(two_hand_moves)

        if self.exp_type == ExpType.GOAL or self.exp_type == ExpType.ACTION:
            all_plans = self._remove_second_move_grab(all_plans)

        return self._remove_duplicates(all_plans)

    def _run_g_file_generator(self):
        if ConfigUtils.DEBUG_PLANNER:
            return

        init_filename = self._get_init_file_path()
        final_filename = self._get_fin_file_path()
        if self.exp_trial_num == '7_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [3], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '7_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '8_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '8_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [15], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '9_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '9_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [8], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '11_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [6, 6], [GFileGenerator.YELLOW_COLOR, GFileGenerator.GREEN_COLOR],
                                       [GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER])
        elif self.exp_trial_num == '11_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [3, 3, 3, 3],
                                       [GFileGenerator.YELLOW_COLOR, GFileGenerator.YELLOW_COLOR,
                                        GFileGenerator.GREEN_COLOR, GFileGenerator.GREEN_COLOR],
                                       [GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER,
                                        GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER])
        elif self.exp_trial_num == '12_1':
            pass
        elif self.exp_trial_num == '12_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '13_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5, 17], [GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR],
                                       [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '13_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [17, 5], [GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR],
                                       [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '14_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '14_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '15_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '15_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [13], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '16_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '16_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [15], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '17_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '17_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [8], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '18_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5, 5, 3, 2],
                                       [GFileGenerator.GREEN_COLOR, GFileGenerator.YELLOW_COLOR,
                                        GFileGenerator.GREEN_COLOR, GFileGenerator.YELLOW_COLOR],
                                       [GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER,
                                        GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '18_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2, 4, 6, 3],
                                       [GFileGenerator.GREEN_COLOR, GFileGenerator.YELLOW_COLOR,
                                        GFileGenerator.GREEN_COLOR, GFileGenerator.YELLOW_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '19_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '19_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2, 3, 3, 2],
                                       [GFileGenerator.GREEN_COLOR, GFileGenerator.YELLOW_COLOR,
                                        GFileGenerator.GREEN_COLOR, GFileGenerator.YELLOW_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '20_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [21], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '20_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [21], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '21_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [4], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '21_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [14], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '22_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [3], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '22_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [15], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '23_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '23_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [8], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '25_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [6, 6], [GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR],
                                       [GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER])
        elif self.exp_trial_num == '25_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2, 4, 4, 2],
                                       [GFileGenerator.RED_COLOR, GFileGenerator.BLUE_COLOR,
                                        GFileGenerator.RED_COLOR, GFileGenerator.BLUE_COLOR],
                                       [GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER,
                                        GFileGenerator.TABLE_LEFT_CENTER, GFileGenerator.TABLE_RIGHT_CENTER])
        elif self.exp_trial_num == '26_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [4], [GFileGenerator.BLUE_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '26_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [16], [GFileGenerator.BLUE_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '27_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5, 17], [GFileGenerator.PURPLE_COLOR, GFileGenerator.YELLOW_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '27_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [17, 5], [GFileGenerator.PURPLE_COLOR, GFileGenerator.YELLOW_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '28_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [12], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '28_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [12], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '29_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [3], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '29_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [15], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '30_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [3], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '30_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [15], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '31_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '31_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [8], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '32_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2, 4, 4, 2, 3],
                                       [GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR,
                                        GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR,
                                        GFileGenerator.BLUE_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '32_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2, 4, 4, 2, 3],
                                       [GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR,
                                        GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR,
                                        GFileGenerator.BLUE_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '33_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [12], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '33_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [2, 4, 4, 2],
                                       [GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR,
                                        GFileGenerator.BLUE_COLOR, GFileGenerator.RED_COLOR],
                                       [GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER,
                                        GFileGenerator.TABLE_CENTER, GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '34_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [17], [GFileGenerator.BLUE_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '34_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [17], [GFileGenerator.BLUE_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '51_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '52_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '53_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [9], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '54_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [5], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '55_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '56_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [10], [GFileGenerator.RED_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '57_1':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [3], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])
        elif self.exp_trial_num == '57_2':
            GFileGenerator.init_g_file(init_filename, final_filename, self.exp_trial_num,
                                       [8], [GFileGenerator.DEFAULT_COLOR], [GFileGenerator.TABLE_CENTER])

        self._execute_g_file_generator(init_filename)
        trial_names_special = ["13_1", "13_2", "27_1", "27_2", "18_1", "15_2"]
        if self.exp_trial_num in trial_names_special:
            self._execute_g_file_generator(final_filename)

    def _execute_g_file_generator(self, file_path):
        arguments_list = [file_path, "#".join(self._block_names_list), file_path, "0"]
        arguments_string = " ".join(arguments_list)
        CPPUtils.run_stability(arguments_string)

    def run_experiment(self, first_block_names_to_grab):
        if first_block_names_to_grab is not None:
            if len(first_block_names_to_grab) == 2:
                planner_output = self._plan_two_hand_moves(first_block_names_to_grab)
            else:
                planner_output = self._plan_all_possible_moves(first_block_names_to_grab)
        else:
            planner_output = self._plan_all_possible_moves(first_block_names_to_grab)

        self._print_planner_output(planner_output[::-1])

        if planner_output is None:
            Msg.print_warn("WARN []: nothing to run for " + str(self.exp_type) + "_" + str(self.exp_trial_num))


class GeneaPlanner:
    def __init__(self, exp_type, exp_trial_num, is_diff,
                 init_blocks_list, max_steps_per_plan, folder_counter, is_one_hand_only_plan):
        self._exp_type = exp_type
        self._exp_trial_num = exp_trial_num
        self._is_diff = is_diff

        self._init_blocks_list = init_blocks_list
        self._max_steps_per_plan = max_steps_per_plan

        self._folder_counter = folder_counter
        self._is_one_hand_only_plan = is_one_hand_only_plan

        self._world_states_tree = None

    def _init_planner_and_world_states(self, first_block_name_to_grab=None):
        self._world_states_tree = WorldStateTree(self._exp_type,
                                                 self._exp_trial_num,
                                                 self._is_diff,
                                                 self._init_blocks_list,
                                                 self._max_steps_per_plan,
                                                 self._folder_counter,
                                                 first_block_names_to_grab=first_block_name_to_grab,
                                                 is_one_hand_only_plan=self._is_one_hand_only_plan)

    def get_all_valid_moves(self):
        if self._world_states_tree is None:
            Msg.print_error("ERROR [get_all_valid_moves]: must run the planner first")
            assert False

        return self._world_states_tree.get_all_valid_moves()

    def get_all_valid_tree_paths(self):
        if self._world_states_tree is None:
            Msg.print_error("ERROR [get_tree_paths_per_valid_move]: must run the planner first")
            assert False

        return self._world_states_tree.get_all_valid_tree_paths()

    def get_ke_per_valid_move(self):
        if self._world_states_tree is None:
            Msg.print_error("ERROR [get_ke_per_valid_move]: must run the planner first")
            assert False

        return self._world_states_tree.get_ke_per_valid_move()

    def get_stability_per_valid_move(self):
        if self._world_states_tree is None:
            Msg.print_error("ERROR [get_stability_per_valid_move]: must run the planner first")
            assert False

        return self._world_states_tree.get_stability_per_valid_move()

    def run_planner(self, first_block_name_to_grab=None):
        self._init_planner_and_world_states(first_block_name_to_grab=first_block_name_to_grab)
        self._world_states_tree.start_planner()
