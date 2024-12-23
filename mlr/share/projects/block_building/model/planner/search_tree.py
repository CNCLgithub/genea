from __future__ import annotations

import copy
import random
from collections import deque, OrderedDict
from itertools import product, combinations
from shutil import copyfile

import numpy as np

from mlr.share.projects.block_building.model.blocks import Block
from mlr.share.projects.block_building.model.planner.agent import RobotActions, RobotHand
from mlr.share.projects.block_building.model.planner.env import Table
from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.core_utils import NameUtils, ConfigUtils
from mlr.share.projects.block_building.utils.cpp_utils import CPPUtils
from mlr.share.projects.block_building.utils.file_utils import GFileUtils, FileUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class RobotActionRecord:
    def __init__(self, robot_action, block_name=None):
        self._robot_action = robot_action
        self._block_name = block_name
        self._table_place = None
        self._fix_above_block_name = None

    def set_robot_action(self, robot_action):
        self._robot_action = robot_action

    def set_table_place(self, table_place):
        self._table_place = table_place

    def set_fix_above_block_name(self, fix_above_block_name):
        self._fix_above_block_name = fix_above_block_name

    def get_robot_action(self):
        return self._robot_action

    def get_block_name(self):
        return self._block_name

    def get_table_place(self):
        return self._table_place

    def get_fix_above_block_name(self):
        return self._fix_above_block_name

    def get_record_as_string(self):
        def get_generic_string():
            action = self._robot_action
            if self._robot_action == RobotActions.GRAB_LEFT_HAND:
                action = "GL"
            elif self._robot_action == RobotActions.GRAB_RIGHT_HAND:
                action = "GR"
            elif self._robot_action == RobotActions.PLACE_LEFT_HAND:
                action = "PL"
            elif self._robot_action == RobotActions.PLACE_RIGHT_HAND:
                action = "PR"
            elif self._robot_action == RobotActions.FIX_LEFT_HAND:
                action = "FL"
            elif self._robot_action == RobotActions.FIX_RIGHT_HAND:
                action = "FR"
            return action + " " + self._block_name.lower()

        def get_place_string():
            table = self._table_place
            if self._table_place == NameUtils.TABLE:
                table = "T"
            elif self._table_place == NameUtils.TABLE_PLACE_LEFT:
                table = "TL"
            elif self._table_place == NameUtils.TABLE_PLACE_RIGHT:
                table = "TR"
            return get_generic_string() + " " + table

        def get_fix_string():
            return get_generic_string() + " " + self._fix_above_block_name

        if RobotActions.GRAB in self._robot_action:
            return get_generic_string()
        elif self._robot_action == RobotActions.PLACE_LEFT_HAND:
            return get_place_string()
        elif self._robot_action == RobotActions.PLACE_RIGHT_HAND:
            return get_place_string()
        elif self._robot_action == RobotActions.FIX_LEFT_HAND:
            return get_fix_string()
        elif self._robot_action == RobotActions.FIX_RIGHT_HAND:
            return get_fix_string()
        return None

    def __eq__(self, other):
        if not isinstance(other, RobotActionRecord):
            return False

        condition1 = self.get_robot_action() == other.get_robot_action()
        condition2 = self.get_block_name() == other.get_block_name()
        if condition1 and condition2:
            return True
        return False


class WorldStateNode:

    def __init__(self, blocks_list, move_num, variation_num):
        """
        move_num        : the depth of the tree
        variation_num   : the breadth at a particular depth of the tree
        """

        self._blocks_list = blocks_list

        self._move_num = move_num
        self._variation_string = variation_num

        self._left_hand = RobotHand()
        self._right_hand = RobotHand()

        self._left_hand_move_record = RobotActionRecord(RobotActions.IDLE_LEFT)
        self._right_hand_move_record = RobotActionRecord(RobotActions.IDLE_RIGHT)

        self._is_valid_to_explore = False

        self._next_world_states = deque([])

        self._table = Table(self._blocks_list)

    # ------------------------------- GETTERS -------------------------------

    def _get_block_by_name(self, block_name) -> Block:
        for block in self._blocks_list:
            if block.get_block_name() == block_name:
                return block
        Msg.print_error("ERROR [get_block_by_name]: block with the name \"" + block_name + "\" was not found")
        assert False

    def get_move_num(self):
        return self._move_num

    def get_variation_num(self):
        return int(self._variation_string.split("_")[-1])

    def get_variation_string(self):
        return self._variation_string

    def get_parent_variation_num(self):
        parents = self._variation_string.split("_")
        if len(parents) == 1:
            return self.get_variation_num()
        return int(parents[-2])

    def get_parent_variation_string(self):
        parents = self._variation_string.split("_")
        if len(parents) == 1:
            return self.get_variation_string()
        return "_".join(parents[:-1])

    def get_all_block_names(self):
        return [block.get_block_name() for block in self._blocks_list]

    def get_num_of_blocks_above(self, block_name):
        block = self._get_block_by_name(block_name)
        if block.get_num_of_blocks_on_top() == 0:
            return 0

        block_above_name = block.get_name_of_block_above()
        return block.get_num_of_blocks_on_top() + self.get_num_of_blocks_above(block_above_name)

    def get_list_of_blocks_above(self, block_name):  # includes current block
        block = self._get_block_by_name(block_name)

        stack_of_blocks_list = [block.get_block_name()]
        if block.get_num_of_blocks_on_top() == 0:
            return stack_of_blocks_list

        block_above_name = block.get_name_of_block_above()
        return stack_of_blocks_list + self.get_list_of_blocks_above(block_above_name)

    def get_current_block_params_dict(self):
        current_block_params_dict = OrderedDict()
        for block in self._blocks_list:
            current_block_params_dict[block.get_block_name()] = block.get_current_block_params()
        return current_block_params_dict

    def get_next_moves_to_process(self, is_diff):
        left_robot_action = self._left_hand_move_record.get_robot_action()
        right_robot_action = self._right_hand_move_record.get_robot_action()

        if is_diff:
            possible_actions_left = RobotActions.get_next_possible_actions_diff(left_robot_action)
            possible_actions_right = RobotActions.get_next_possible_actions_diff(right_robot_action)
            return list(product(possible_actions_left, possible_actions_right))

        possible_actions_left = RobotActions.get_next_possible_actions(left_robot_action)
        possible_actions_right = RobotActions.get_next_possible_actions(right_robot_action)
        return list(product(possible_actions_left, possible_actions_right))

    def get_available_blocks_to_grab(self, is_diff):
        blocks_remaining_list = []
        for block in self._blocks_list:
            if block.is_available() or block.is_placed():
                blocks_remaining_list.append(block)
        if is_diff:
            return sorted(blocks_remaining_list, key=lambda x: (x.get_final_pos().z()))
        return sorted(blocks_remaining_list, key=lambda x: x.get_block_name())

    def get_available_block_names_to_grab(self, is_diff):
        block_names_remaining_list = []
        for block in self.get_available_blocks_to_grab(is_diff):
            block_names_remaining_list.append(block.get_block_name())
        return block_names_remaining_list

    def get_action_details_as_string(self, delimiter=", "):
        left_hand_action_details = self._left_hand_move_record.get_record_as_string()
        right_hand_action_details = self._right_hand_move_record.get_record_as_string()

        return_string = []
        if left_hand_action_details is not None:
            return_string.append(left_hand_action_details)
        if right_hand_action_details is not None:
            return_string.append(right_hand_action_details)

        if len(return_string) > 0:
            return delimiter.join(return_string)
        return None

    def get_contents_as_string(self, delimiter=", "):
        left_hand_content = self._left_hand_move_record.get_block_name()
        right_hand_content = self._right_hand_move_record.get_block_name()

        left_hand_action = self._left_hand_move_record.get_robot_action()
        right_hand_action = self._right_hand_move_record.get_robot_action()

        return_string = []
        if left_hand_content is not None and not left_hand_action == RobotActions.FIX_LEFT_HAND:
            return_string.append(left_hand_content)
        if right_hand_content is not None and not right_hand_action == RobotActions.FIX_RIGHT_HAND:
            return_string.append(right_hand_content)

        if len(return_string) > 0:
            return delimiter.join(return_string)
        return "None"

    def get_next_world_states(self):
        return self._next_world_states

    def get_best_hand_for_grab(self, block_name, custom_optimize_status=None):
        do_optimize = ConfigUtils.DO_GRAB_OPTIMIZATION
        if custom_optimize_status is not None:
            do_optimize = custom_optimize_status

        if do_optimize:
            block = self._get_block_by_name(block_name)

            check_left = False
            check_right = False
            if block.is_placed():
                if block.is_placed_left():
                    check_left = True
                elif block.is_placed_right():
                    check_right = True
            elif block.is_available():
                block_y_current = block.get_current_pos().y()
                block_y_final = block.get_final_pos().y()

                if -0.05 < block_y_current < 0.05:  # if block in center, choose hand based on final position
                    if block_y_final <= 0.0:
                        check_left = True
                    if block_y_final > 0.0:
                        check_right = True
                else:
                    if block_y_current <= 0.0:
                        check_left = True
                    if block_y_current > 0.0:
                        check_right = True
            else:
                Msg.print_error("ERROR [get_best_hand_for_grab]: block to grab is in an invalid state")
                assert False

            if check_left and self._left_hand.is_available():
                return NameUtils.ROBOT_HAND_LEFT
            if check_right and self._right_hand.is_available():
                return NameUtils.ROBOT_HAND_RIGHT

        if self._left_hand.is_available():
            return NameUtils.ROBOT_HAND_LEFT
        elif self._right_hand.is_available():
            return NameUtils.ROBOT_HAND_RIGHT

        Msg.print_error("ERROR [get_best_hand_for_grab]: hm, something really weird happened...")
        assert False

    def get_best_hands_for_two_grabs(self, block_name1, block_name2):
        robot_hand1 = self.get_best_hand_for_grab(block_name1, custom_optimize_status=True)
        robot_hand2 = self.get_best_hand_for_grab(block_name2, custom_optimize_status=True)

        if not robot_hand1 == robot_hand2:
            return robot_hand1, robot_hand2

        if robot_hand1 == NameUtils.ROBOT_HAND_LEFT:
            return robot_hand1, NameUtils.ROBOT_HAND_RIGHT

        if robot_hand1 == NameUtils.ROBOT_HAND_RIGHT:
            return robot_hand1, NameUtils.ROBOT_HAND_LEFT

        Msg.print_error("ERROR [get_best_hand_for_grab]: hm, something really weird happened...")
        assert False

    def get_left_hand_action_record(self) -> RobotActionRecord:
        return self._left_hand_move_record

    def get_right_hand_action_record(self) -> RobotActionRecord:
        return self._right_hand_move_record

    def get_block_height_by_block_name(self, block_name):
        return self._get_block_by_name(block_name).get_current_pos().z()

    def get_sum_hand_content_at_final(self):
        left_block_name = self.get_left_hand_action_record().get_block_name()
        right_block_name = self.get_right_hand_action_record().get_block_name()
        return_sum_value = 0.0
        if left_block_name is not None:
            return_sum_value += self._get_block_by_name(left_block_name).get_final_pos().z()
        if right_block_name is not None:
            return_sum_value += self._get_block_by_name(right_block_name).get_final_pos().z()
        return return_sum_value

    # ------------------------------- SETTERS -------------------------------

    def set_move_num(self, move_num):
        self._move_num = move_num

    def set_variation_string(self, variation_string):
        self._variation_string = variation_string

    def set_as_invalid_to_explore(self):
        self._is_valid_to_explore = False

    def set_as_valid_to_explore(self):
        self._is_valid_to_explore = True

    # ------------------------------- CHECKERS -------------------------------
    def is_left_hand_available(self):
        return self._left_hand.is_available()

    def is_right_hand_available(self):
        return self._right_hand.is_available()

    @staticmethod
    def is_block_name_table(block_name):
        if block_name == NameUtils.TABLE:
            return True
        if block_name == NameUtils.TABLE_PLACE_LEFT or block_name == NameUtils.TABLE_PLACE_RIGHT:
            return True
        return False

    def are_actions_active(self):
        if self._is_valid_to_explore:
            return True

        action_left = self._left_hand_move_record.get_robot_action()
        action_right = self._right_hand_move_record.get_robot_action()
        if not RobotActions.are_both_inactive(action_left, action_right):
            return True

        return False

    def is_valid_state(self):
        return self._is_valid_to_explore

    def is_block_name_in_fin_pos(self, block_name):
        block = self._get_block_by_name(block_name)

        if not block.is_block_at_final_pos():
            return False

        block_underneath_name = block.get_name_of_block_underneath()
        if block_underneath_name == NameUtils.TABLE:   # if block is resting on the table
            return True

        return self.is_block_name_in_fin_pos(block_underneath_name)  # check if block below is also at final position

    def are_all_blocks_done(self):
        for block in self._blocks_list:
            if not block.is_block_at_final_pos():
                return False
        return True

    def are_blocks_stacked(self, block_name1, block_name2):
        if block_name1 is None or block_name2 is None:
            Msg.print_info("INFO [are_held_blocks_stacked_at_final]: one of the hands is empty...")
            return False

        block1 = self._get_block_by_name(block_name1)
        block2 = self._get_block_by_name(block_name2)

        block1_above = block1.get_name_of_block_above_at_final()
        block2_above = block2.get_name_of_block_above_at_final()

        if block1_above == block_name2 or block2_above == block_name1:
            return True

        return False

    def are_held_blocks_stacked_at_final(self):
        block1 = self._left_hand.get_contents()
        block2 = self._right_hand.get_contents()

        if block1 is None and block2 is None:
            return False
        return self.are_blocks_stacked(block1.get_block_name(), block2.get_block_name())

    def can_block_be_grabbed(self, block_name, robot_hand=None):
        if self.get_num_of_blocks_above(block_name) > 0:
            Msg.print_info("INFO [grab_block_name]: cannot pick a block that is under another " + block_name)
            return False

        block = self._get_block_by_name(block_name)

        if not block.is_placed() and not block.is_available():
            return False

        if robot_hand is None and not self._left_hand.is_available() and not self._right_hand.is_available():
            Msg.print_info("INFO [can_block_be_grabbed]: cannot grab, all hands are occupied")
            return False

        if robot_hand == NameUtils.ROBOT_HAND_LEFT and not self._left_hand.is_available():
            Msg.print_info("INFO [can_block_be_grabbed]: cannot grab, left hand is occupied")
            return False

        if robot_hand == NameUtils.ROBOT_HAND_RIGHT and not self._right_hand.is_available():
            Msg.print_info("INFO [can_block_be_grabbed]: cannot grab, right hand is occupied")
            return False

        return True

    def can_block_in_hand_be_placed(self, robot_hand):
        if robot_hand == NameUtils.ROBOT_HAND_LEFT and self._left_hand.is_available():
            Msg.print_error("ERROR [can_block_in_hand_be_placed]: operation invalid, left hand has nothing to place")
            assert False
        if robot_hand == NameUtils.ROBOT_HAND_RIGHT and self._right_hand.is_available():
            Msg.print_error("ERROR [can_block_in_hand_be_placed]: operation invalid, right hand has nothing to place")
            assert False

        # do not place if block can be fixed
        if self.can_block_in_hand_be_fixed(robot_hand):
            return False

        if not self._table.is_place_left_available() and not self._table.is_place_right_available():
            return False

        # release block from hand
        block = None
        if robot_hand == NameUtils.ROBOT_HAND_LEFT:
            block = self._left_hand.get_contents()
        elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
            block = self._right_hand.get_contents()

        if block is None:
            Msg.print_error("ERROR [_move_block_from_hand_to_table]: hand with None as content")
            assert False

        if block.has_been_placed():
            return False

        Msg.print_info("INFO [can_block_in_hand_be_placed]: block can be placed!")
        return True

    def can_block_in_hand_be_fixed(self, robot_hand):
        if robot_hand == NameUtils.ROBOT_HAND_LEFT and self._left_hand.is_available():
            Msg.print_error("ERROR [can_block_in_hand_be_fixed]: operation invalid, left hand has nothing to fix")
            assert False
        if robot_hand == NameUtils.ROBOT_HAND_RIGHT and self._right_hand.is_available():
            Msg.print_error("ERROR [can_block_in_hand_be_fixed]: operation invalid, right hand has nothing to fix")
            assert False

        block = None
        if robot_hand == NameUtils.ROBOT_HAND_LEFT:
            block = self._left_hand.get_contents()
        elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
            block = self._right_hand.get_contents()

        if block is None:
            Msg.print_error("ERROR [can_block_in_hand_be_fixed]: hand with None as content")
            assert False

        fin_loc_name = block.get_name_of_block_underneath_at_final()
        if fin_loc_name == NameUtils.TABLE:
            if self._table.is_spot_free(block):
                Msg.print_info("INFO [fix_block_from_hand]: block can be fixed at final location")
                return True
        else:
            is_block_free_above = self._get_block_by_name(fin_loc_name).is_free_to_support()
            if is_block_free_above and self.is_block_name_in_fin_pos(fin_loc_name):
                Msg.print_info("INFO [fix_block_from_hand]: block can be fixed at final location")
                return True
        Msg.print_info("INFO [fix_block_from_hand]: unable to fix block at final location")
        return False

    # ------------------------------- OTHER FUNCTIONS -------------------------------

    def _update_name_of_block_above(self, block_name, block_above_name):
        block = self._get_block_by_name(block_name)
        block.set_name_of_block_above(block_above_name)
        self._update_block(block)

    def _update_block(self, new_block_obj):
        if not isinstance(new_block_obj, Block):
            Msg.print_error("ERROR [update_block]: invalid object was passed")
            assert False

        for index, block_obj in enumerate(self._blocks_list):
            if block_obj.get_block_name() == new_block_obj.get_block_name():
                self._blocks_list[index] = new_block_obj
                break

        self._table.update_occupied_spots_list(self._blocks_list)

    def make_placed_blocks_available(self):
        for block in self._blocks_list:
            if block.is_placed_but_unavailable():
                while not block.is_available() and not block.is_placed():
                    block.make_block_available()

    def update_blocks_state(self):
        for block in self._blocks_list:
            if not self.is_block_name_in_fin_pos(block.get_block_name()):
                block.make_block_available()

    def update_current_block_params(self, new_block_params_dict: dict):
        for block in self._blocks_list:
            new_block_params = new_block_params_dict[block.get_block_name()]
            if not block.is_grabbed() and not block.is_placed():
                block.update_current_block_params(new_block_params)

        self._table.update_occupied_spots_list(self._blocks_list)

    def update_final_block_position(self, new_block_params_dict, block_name):
        for block in self._blocks_list:
            if block.get_block_name() == block_name and block.is_block_at_final_pos():
                new_position = new_block_params_dict[block_name].get_position()
                if abs(new_position.z() - block.get_final_pos().z()) < ConfigUtils.NOISE_THRESHOLD:
                    block.update_final_block_position(new_position)

        self._table.update_occupied_spots_list(self._blocks_list)

    def _move_block_name_to_hand(self, block_name, robot_hand=NameUtils.ROBOT_HAND_LEFT):
        block = self._get_block_by_name(block_name)

        # update status of the block underneath
        block_underneath_name = block.get_name_of_block_underneath()
        if not self.is_block_name_table(block_underneath_name):
            self._update_name_of_block_above(block_underneath_name, NameUtils.NO_BLOCK_ON_TOP)

        # move the block to the hand
        if robot_hand == NameUtils.ROBOT_HAND_LEFT:
            block.move_to_left_hand()
            self._left_hand.grab_block(block)
            self._left_hand_move_record = RobotActionRecord(RobotActions.GRAB_LEFT_HAND, block.get_block_name())
        elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
            block.move_to_right_hand()
            self._right_hand.grab_block(block)
            self._right_hand_move_record = RobotActionRecord(RobotActions.GRAB_RIGHT_HAND, block.get_block_name())

        self._update_block(block)

    def _move_block_from_hand_to_table(self,
                                       robot_hand=NameUtils.ROBOT_HAND_LEFT,
                                       place_on_table=NameUtils.TABLE_PLACE_LEFT):
        # release block from hand
        block = None
        if robot_hand == NameUtils.ROBOT_HAND_LEFT:
            block = self._left_hand.get_contents()
            self._left_hand.release_block()
            self._left_hand_move_record = RobotActionRecord(RobotActions.PLACE_LEFT_HAND, block.get_block_name())
        elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
            block = self._right_hand.get_contents()
            self._right_hand.release_block()
            self._right_hand_move_record = RobotActionRecord(RobotActions.PLACE_RIGHT_HAND, block.get_block_name())

        # move block to place
        if place_on_table == NameUtils.TABLE_PLACE_LEFT:
            block.move_to_place_left()
            if robot_hand == NameUtils.ROBOT_HAND_LEFT:
                self._left_hand_move_record.set_table_place(NameUtils.TABLE_PLACE_LEFT)
            elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
                self._right_hand_move_record.set_table_place(NameUtils.TABLE_PLACE_LEFT)
        elif place_on_table == NameUtils.TABLE_PLACE_RIGHT:
            block.move_to_place_right()
            if robot_hand == NameUtils.ROBOT_HAND_LEFT:
                self._left_hand_move_record.set_table_place(NameUtils.TABLE_PLACE_RIGHT)
            elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
                self._right_hand_move_record.set_table_place(NameUtils.TABLE_PLACE_RIGHT)
        self._update_block(block)

    def _move_block_name_to_fin_pos(self, robot_hand=NameUtils.ROBOT_HAND_LEFT):
        # release block from hand
        block = None
        if robot_hand == NameUtils.ROBOT_HAND_LEFT:
            block = self._left_hand.get_contents()
            self._left_hand.release_block()
            self._left_hand_move_record = RobotActionRecord(RobotActions.FIX_LEFT_HAND, block.get_block_name())
            self._left_hand_move_record.set_fix_above_block_name(block.get_name_of_block_underneath_at_final())
        elif robot_hand == NameUtils.ROBOT_HAND_RIGHT:
            block = self._right_hand.get_contents()
            self._right_hand.release_block()
            self._right_hand_move_record = RobotActionRecord(RobotActions.FIX_RIGHT_HAND, block.get_block_name())
            self._right_hand_move_record.set_fix_above_block_name(block.get_name_of_block_underneath_at_final())

        if block is None:
            Msg.print_error("ERROR [_move_block_name_to_fin_pos]: hand with None as content")
            assert False

        # move the block to the final position
        block.move_to_final_pos()
        self._update_block(block)

        # update the status of the new block that is underneath
        block_underneath_name = block.get_name_of_block_underneath()
        if not self.is_block_name_table(block_underneath_name):
            self._update_name_of_block_above(block_underneath_name, block.get_block_name())

    # ------------------------------- CORE FUNCTIONS -------------------------------
    def grab_block_name(self, block_name, robot_hand=None):
        if not self.can_block_be_grabbed(block_name):
            return False

        if robot_hand is None:
            robot_hand = self.get_best_hand_for_grab(block_name)

        if robot_hand == NameUtils.ROBOT_HAND_LEFT and self._left_hand.is_available():
            self._move_block_name_to_hand(block_name, NameUtils.ROBOT_HAND_LEFT)
            return True
        elif robot_hand == NameUtils.ROBOT_HAND_RIGHT and self._right_hand.is_available():
            self._move_block_name_to_hand(block_name, NameUtils.ROBOT_HAND_RIGHT)
            return True

        Msg.print_info("INFO [grab_block_name]: all hands are occupied, cull tree here")
        return False

    def place_block_from_hand(self, robot_hand=NameUtils.ROBOT_HAND_LEFT):
        if not self.can_block_in_hand_be_placed(robot_hand):
            return False

        # determine where on table should the block be placed
        table_place = None
        if robot_hand == NameUtils.ROBOT_HAND_LEFT:
            if self._table.is_place_left_available():
                table_place = NameUtils.TABLE_PLACE_LEFT
            elif self._table.is_place_right_available():
                table_place = NameUtils.TABLE_PLACE_RIGHT

        if robot_hand == NameUtils.ROBOT_HAND_RIGHT:
            if self._table.is_place_right_available():
                table_place = NameUtils.TABLE_PLACE_RIGHT
            elif self._table.is_place_left_available():
                table_place = NameUtils.TABLE_PLACE_LEFT

        if table_place is not None:
            self._move_block_from_hand_to_table(robot_hand, table_place)
            return True

        Msg.print_info("INFO [place_block_from_hand]: all places on the table are occupied, cull tree here")
        return False

    def fix_block_from_hand(self, robot_hand=NameUtils.ROBOT_HAND_LEFT):
        if not self.can_block_in_hand_be_fixed(robot_hand):
            return False

        self._move_block_name_to_fin_pos(robot_hand)

        return True

    def skip_move(self, robot_action):
        if robot_action == RobotActions.DO_NOTHING_G_LEFT_HAND:
            self._left_hand_move_record.set_robot_action(robot_action)
        elif robot_action == RobotActions.DO_NOTHING_G_RIGHT_HAND:
            self._right_hand_move_record.set_robot_action(robot_action)
        elif robot_action == RobotActions.IDLE_LEFT:
            self._left_hand_move_record = RobotActionRecord(robot_action)
        elif robot_action == RobotActions.IDLE_RIGHT:
            self._right_hand_move_record = RobotActionRecord(robot_action)
        return True

    def execute_action(self, action, block_name=None):
        if RobotActions.GRAB in action and block_name is None:
            Msg.print_error("ERROR [execute_action]: no block name was specified for grab action")
            assert False

        if action == RobotActions.GRAB:
            return self.grab_block_name(block_name)
        elif action == RobotActions.GRAB_LEFT_HAND:
            return self.grab_block_name(block_name, NameUtils.ROBOT_HAND_LEFT)
        elif action == RobotActions.GRAB_RIGHT_HAND:
            return self.grab_block_name(block_name, NameUtils.ROBOT_HAND_RIGHT)
        elif action == RobotActions.PLACE_LEFT_HAND:
            return self.place_block_from_hand(NameUtils.ROBOT_HAND_LEFT)
        elif action == RobotActions.PLACE_RIGHT_HAND:
            return self.place_block_from_hand(NameUtils.ROBOT_HAND_RIGHT)
        elif action == RobotActions.FIX_LEFT_HAND:
            return self.fix_block_from_hand(NameUtils.ROBOT_HAND_LEFT)
        elif action == RobotActions.FIX_RIGHT_HAND:
            return self.fix_block_from_hand(NameUtils.ROBOT_HAND_RIGHT)
        else:
            return self.skip_move(action)

    def add_next_world_state(self, world_state_node):
        self._next_world_states.append(world_state_node)


class WorldStateTree:
    def __init__(self, exp_type, exp_trial_num, is_diff,
                 init_blocks_list,
                 max_steps_per_plan,
                 folder_counter,
                 first_block_names_to_grab=None,
                 is_one_hand_only_plan=False):

        self._exp_type = exp_type
        self._exp_trial_num = exp_trial_num

        self._is_diff = is_diff
        self._core_identifier = self._exp_type + PathUtils.UNDERSCORE + self._exp_trial_num

        self._init_blocks_list = sorted(init_blocks_list, key=lambda x: x.get_block_name())

        self._folder_counter = folder_counter
        self._intermediate_folder_name = self._core_identifier + PathUtils.UNDERSCORE + str(self._folder_counter)

        self._max_steps_per_plan = max_steps_per_plan

        self._first_block_names_to_grab = first_block_names_to_grab

        self._is_one_hand_only_plan = is_one_hand_only_plan

        self._root_world_state = None
        self._world_states_to_process = deque([])

        self._block_names_list = None
        self._sign_by_height_x = None
        self._sign_by_height_y = None

        self._init_world_state_tree()

    @staticmethod
    def _get_new_world_state(world_state, move_num, variation_num):
        world_state_node = copy.deepcopy(world_state)
        world_state_node.set_move_num(move_num)
        world_state_node.set_variation_string(str(variation_num))
        world_state_node.set_as_invalid_to_explore()
        world_state_node.update_blocks_state()
        return world_state_node

    @staticmethod
    def _get_new_var_string(old_num, new_num):
        if str(old_num) == "":
            return str(new_num)
        return str(old_num) + PathUtils.UNDERSCORE + str(new_num)

    @staticmethod
    def _get_intermediate_filename(file_type, variation_string) -> str:
        return file_type + PathUtils.UNDERSCORE + str(variation_string)

    def _get_intermediate_state_filename(self, variation_string) -> str:
        return self._get_intermediate_filename("state", variation_string) + PathUtils.DOT_G

    def _get_intermediate_ke_filename(self, variation_string):
        return self._get_intermediate_filename("ke", variation_string) + PathUtils.DOT_TXT

    def _get_intermediate_core_dir_path(self) -> str:
        return PathUtils.join(PathUtils.get_intermediate_files_dirpath(), self._intermediate_folder_name)

    def _get_block_names_list(self):
        if self._block_names_list is not None:
            return self._block_names_list

        self._block_names_list = []
        for block in self._init_blocks_list:
            self._block_names_list.append(block.get_block_name())
        return self._block_names_list

    def _get_new_blocks_params_from_intermediate_file(self, variation_str):
        file_path = PathUtils.join(self._get_intermediate_core_dir_path(),
                                   self._get_intermediate_state_filename(variation_str))
        block_names_list = self._get_block_names_list()

        return GFileUtils.get_block_params_from_parse_g_file_path(file_path, block_names_list)

    def get_first_move_world_states(self):
        blocks_remaining = []
        if self._first_block_names_to_grab is not None:
            for block in self._init_blocks_list:
                if block.get_block_name() in self._first_block_names_to_grab:
                    blocks_remaining.append(block)

            if len(blocks_remaining) == 1:
                return self._get_one_hand_only_grabs(self._root_world_state,
                                                     move_num=1, parent_var_string="", var_num=1, is_first_move=True)[0]

            next_world_states = []
            self.check_and_execute_action(self._root_world_state, 1, 1,
                                          RobotActions.GRAB_LEFT_HAND, self._first_block_names_to_grab[0],
                                          RobotActions.GRAB_RIGHT_HAND, self._first_block_names_to_grab[1],
                                          next_world_states, do_assignment=False)
            return next_world_states

        next_world_states = []
        var_num = 1
        if self._is_one_hand_only_plan:
            next_world_states, var_num = self._get_one_hand_only_grabs(self._root_world_state, move_num=1,
                                                                       parent_var_string="", var_num=var_num)
        else:
            next_world_states.extend(self._get_two_hand_only_grabs(self._root_world_state, move_num=1,
                                                                   parent_var_string="", var_num=var_num)[0])
        return next_world_states

    def get_all_valid_moves(self):
        def _explore_node(node, is_root=False):
            if not node.is_valid_state():
                return

            action = node.get_action_details_as_string()
            if not is_root and action is None:
                Msg.print_info("INFO [_explore_node]: reached a node with no valid moves")
                return

            if is_root:
                action = "START"

            next_world_states = node.get_next_world_states()
            if len(next_world_states) > 0:
                for world_state in next_world_states:
                    if not world_state.is_valid_state():
                        continue
                    for returned_moves in _explore_node(world_state):
                        yield [action] + returned_moves
            else:
                yield [action]

        if not self._root_world_state.is_valid_state():
            Msg.print_warn("WARN [_explore_node]: nothing to return -- no moves found")
            return [""]

        return list(_explore_node(self._root_world_state,  is_root=True))

    def get_moves_from_variation_num(self, variation_num):  # added solely for debugging purpose
        all_moves_list = []

        var_num_by_depth = variation_num.split(PathUtils.UNDERSCORE)

        child_world_states = self._root_world_state.get_next_world_states()
        for var_num in var_num_by_depth:
            for query_world_state in child_world_states:
                if query_world_state.get_variation_num() == int(var_num):
                    all_moves_list.append(query_world_state.get_action_details_as_string())
                    child_world_states = query_world_state.get_next_world_states()

        return all_moves_list

    def get_all_valid_tree_paths(self):
        def _explore_node(node: WorldStateNode, is_root=False):
            if not node.is_valid_state():
                return

            return_value = str(self._folder_counter)
            if not is_root:
                return_value = str(node.get_variation_num())

            next_world_states = node.get_next_world_states()
            if len(next_world_states) > 0:
                for world_state in next_world_states:
                    if not world_state.is_valid_state():
                        continue
                    for returned_ke_values in _explore_node(world_state):
                        yield [return_value] + returned_ke_values
            else:
                yield [return_value]

        if not self._root_world_state.is_valid_state():
            Msg.print_warn("WARN [_explore_node]: nothing to return -- no moves found")
            return [""]

        return list(_explore_node(self._root_world_state,  is_root=True))

    def get_ke_per_valid_move(self):
        def _get_ke_value_from_file(world_state_node):
            ke_filename = PathUtils.join(self._get_intermediate_core_dir_path(),
                                         self._get_intermediate_ke_filename(world_state_node.get_variation_string()))
            with open(ke_filename) as ke_file:
                for line in ke_file:
                    return float(line.split(",")[0])
                Msg.print_error("ERROR [_get_ke_value_from_file]: no KE value printed in file")
                assert False

        def _explore_node(node: WorldStateNode, is_root=False):
            if not node.is_valid_state():
                return

            ke_value = 0.0
            if not is_root:
                ke_value = _get_ke_value_from_file(node)

            next_world_states = node.get_next_world_states()
            if len(next_world_states) > 0:
                for world_state in next_world_states:
                    if not world_state.is_valid_state():
                        continue
                    for returned_ke_values in _explore_node(world_state):
                        yield [ke_value] + returned_ke_values
            else:
                yield [ke_value]

        if not self._root_world_state.is_valid_state():
            Msg.print_warn("WARN [_explore_node]: nothing to return -- no moves found")
            return [""]

        return list(_explore_node(self._root_world_state,  is_root=True))

    def get_stability_per_valid_move(self):
        def _get_stability_value_from_file(world_state_node):
            s_filename = PathUtils.join(self._get_intermediate_core_dir_path(),
                                        self._get_intermediate_ke_filename(world_state_node.get_variation_string()))
            with open(s_filename) as stability_file:
                for line in stability_file:
                    return float(line.split(",")[1])
                Msg.print_error("ERROR [_get_ke_value_from_file]: no KE value printed in file")
                assert False

        def _explore_node(node: WorldStateNode, is_root=False):
            if not node.is_valid_state():
                return

            stability = 0.0
            if not is_root:
                stability = _get_stability_value_from_file(node)

            next_world_states = node.get_next_world_states()
            if len(next_world_states) > 0:
                for world_state in next_world_states:
                    if not world_state.is_valid_state():
                        continue
                    for returned_stability_values in _explore_node(world_state):
                        yield [stability] + returned_stability_values
            else:
                yield [stability]

        if not self._root_world_state.is_valid_state():
            Msg.print_warn("WARN [_explore_node]: nothing to return -- no moves found")
            return [""]

        return list(_explore_node(self._root_world_state,  is_root=True))

    def get_root_world_state(self) -> WorldStateNode:
        return self._root_world_state

    def get_world_state_list_from_root(self, variation_string):
        world_state_list = []

        var_num_by_depth = variation_string.split(PathUtils.UNDERSCORE)

        child_world_states = self.get_root_world_state().get_next_world_states()
        for var_num in var_num_by_depth:
            for query_world_state in child_world_states:
                if query_world_state.get_variation_num() == int(var_num):
                    world_state_list.append(query_world_state)
                    child_world_states = query_world_state.get_next_world_states()

        return world_state_list

    @staticmethod
    def _get_record(action1, block_name1, action2, block_name2):
        record1 = RobotActionRecord(robot_action=action1)
        record2 = RobotActionRecord(robot_action=action2)
        if block_name1 is not None:
            record1 = RobotActionRecord(robot_action=action1, block_name=block_name1)

        if block_name2 is not None:
            record2 = RobotActionRecord(robot_action=action2, block_name=block_name2)
        return record1, record2

    def _get_moves_to_remove(self, curr_world_state: WorldStateNode):
        records_to_remove = []

        left_record = curr_world_state.get_left_hand_action_record()
        right_record = curr_world_state.get_right_hand_action_record()

        left_hand_action = left_record.get_robot_action()
        right_hand_action = right_record.get_robot_action()

        left_hand_block = left_record.get_block_name()
        right_hand_block = right_record.get_block_name()

        if RobotActions.GRAB in left_hand_action and RobotActions.GRAB in right_hand_action:
            records_to_remove.append(self._get_record(RobotActions.GRAB_LEFT_HAND, left_hand_block,
                                                      RobotActions.IDLE_RIGHT, None))
            records_to_remove.append(self._get_record(RobotActions.GRAB_LEFT_HAND, left_hand_block,
                                                      RobotActions.DO_NOTHING_G_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.IDLE_LEFT, None,
                                                      RobotActions.GRAB_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.DO_NOTHING_G_LEFT_HAND, left_hand_block,
                                                      RobotActions.GRAB_RIGHT_HAND, right_hand_block))

        elif RobotActions.PLACE in left_hand_action and RobotActions.PLACE in right_hand_action:
            records_to_remove.append(self._get_record(RobotActions.PLACE_LEFT_HAND, left_hand_block,
                                                      RobotActions.IDLE_RIGHT, None))
            records_to_remove.append(self._get_record(RobotActions.PLACE_LEFT_HAND, left_hand_block,
                                                      RobotActions.DO_NOTHING_G_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.IDLE_LEFT, None,
                                                      RobotActions.PLACE_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.DO_NOTHING_G_LEFT_HAND, left_hand_block,
                                                      RobotActions.PLACE_RIGHT_HAND, right_hand_block))

        elif RobotActions.FIX in left_hand_action and RobotActions.FIX in right_hand_action:
            records_to_remove.append(self._get_record(RobotActions.FIX_LEFT_HAND, left_hand_block,
                                                      RobotActions.IDLE_RIGHT, None))
            records_to_remove.append(self._get_record(RobotActions.FIX_LEFT_HAND, left_hand_block,
                                                      RobotActions.DO_NOTHING_G_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.FIX_LEFT_HAND, left_hand_block,
                                                      RobotActions.PLACE_RIGHT_HAND, right_hand_block))

            records_to_remove.append(self._get_record(RobotActions.PLACE_LEFT_HAND, left_hand_block,
                                                      RobotActions.IDLE_RIGHT, None))
            records_to_remove.append(self._get_record(RobotActions.PLACE_LEFT_HAND, left_hand_block,
                                                      RobotActions.DO_NOTHING_G_RIGHT_HAND, right_hand_block))

            records_to_remove.append(self._get_record(RobotActions.IDLE_LEFT, None,
                                                      RobotActions.FIX_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.DO_NOTHING_G_LEFT_HAND, left_hand_block,
                                                      RobotActions.FIX_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.PLACE_LEFT_HAND, left_hand_block,
                                                      RobotActions.FIX_RIGHT_HAND, right_hand_block))

            records_to_remove.append(self._get_record(RobotActions.IDLE_LEFT, None,
                                                      RobotActions.PLACE_RIGHT_HAND, right_hand_block))
            records_to_remove.append(self._get_record(RobotActions.DO_NOTHING_G_LEFT_HAND, left_hand_block,
                                                      RobotActions.PLACE_RIGHT_HAND, right_hand_block))

        else:
            if RobotActions.GRAB in left_hand_action and not RobotActions.is_inactive(right_hand_action):
                records_to_remove.append(self._get_record(RobotActions.GRAB_LEFT_HAND, left_hand_block,
                                                          RobotActions.IDLE_RIGHT, None))
                records_to_remove.append(self._get_record(RobotActions.GRAB_LEFT_HAND, left_hand_block,
                                                          RobotActions.DO_NOTHING_G_RIGHT_HAND, right_hand_block))

            if RobotActions.GRAB in right_hand_action and not RobotActions.is_inactive(left_hand_action):
                records_to_remove.append(self._get_record(RobotActions.IDLE_LEFT, None,
                                                          RobotActions.GRAB_RIGHT_HAND, right_hand_block))
                records_to_remove.append(self._get_record(RobotActions.DO_NOTHING_G_LEFT_HAND, left_hand_block,
                                                          RobotActions.GRAB_RIGHT_HAND, right_hand_block))

            if not RobotActions.is_inactive(left_hand_action) and not RobotActions.is_inactive(right_hand_action):
                if RobotActions.FIX in left_hand_action or RobotActions.FIX in right_hand_action:
                    return records_to_remove

                if not right_hand_action == RobotActions.IDLE_RIGHT:
                    records_to_remove.append(self._get_record(left_hand_action, left_hand_block,
                                                              RobotActions.IDLE_RIGHT, None))

                if not left_hand_action == RobotActions.IDLE_LEFT:
                    records_to_remove.append(self._get_record(RobotActions.IDLE_LEFT, None,
                                                              right_hand_action, right_hand_block))

        return records_to_remove

    def _get_one_hand_only_grabs(self, world_state: WorldStateNode, move_num, parent_var_string, var_num,
                                 is_first_move=False):
        next_world_states = []
        for block in world_state.get_available_blocks_to_grab(self._is_diff):
            if is_first_move and not block.get_block_name() in self._first_block_names_to_grab:
                continue

            robot_hand = world_state.get_best_hand_for_grab(block.get_block_name())

            left_hand_action = RobotActions.GRAB_LEFT_HAND
            left_hand_block_name = block.get_block_name()
            right_hand_action = RobotActions.IDLE_RIGHT
            right_hand_block_name = None
            if robot_hand == NameUtils.ROBOT_HAND_RIGHT:
                left_hand_action = RobotActions.IDLE_LEFT
                left_hand_block_name = None
                right_hand_action = RobotActions.GRAB_RIGHT_HAND
                right_hand_block_name = block.get_block_name()

            var_string = self._get_new_var_string(parent_var_string, var_num)
            if self.check_and_execute_action(world_state, move_num, var_string,
                                             left_hand_action, left_hand_block_name,
                                             right_hand_action, right_hand_block_name,
                                             next_world_states):
                var_num += 1

        return next_world_states, var_num

    def _get_two_hand_only_grabs(self, world_state: WorldStateNode, move_num, parent_var_string, var_num):
        next_world_states = []
        for block_pairs in list(combinations(world_state.get_available_blocks_to_grab(self._is_diff), 2)):
            var_string = self._get_new_var_string(parent_var_string, var_num)
            if self.check_and_execute_action(world_state, move_num, var_string,
                                             RobotActions.GRAB, block_pairs[0].get_block_name(),
                                             RobotActions.GRAB, block_pairs[1].get_block_name(),
                                             next_world_states):
                var_num += 1

        return next_world_states, var_num

    def _get_all_possible_re_plan_actions(self, world_state: WorldStateNode):
        move_num = world_state.get_move_num()
        var_num = world_state.get_variation_num()

        curr_var_string = world_state.get_variation_string()
        parent_var_string = world_state.get_parent_variation_string()

        # place any grabbed blocks before we continue
        if not world_state.is_left_hand_available() or not world_state.is_right_hand_available():
            next_world_states = []

            # ------------------------------ PLACE and PLACE ------------------------------
            left_hand_action = RobotActions.IDLE_LEFT
            right_hand_action = RobotActions.IDLE_LEFT
            if not world_state.is_left_hand_available():
                left_hand_action = RobotActions.PLACE_LEFT_HAND
            if not world_state.is_right_hand_available():
                right_hand_action = RobotActions.PLACE_RIGHT_HAND

            move_num = move_num + 1
            var_num = 1
            var_string = self._get_new_var_string(curr_var_string, var_num)
            if self.check_and_execute_action(world_state, move_num, var_string,
                                             left_hand_action, None, right_hand_action, None,
                                             next_world_states):
                for ws in next_world_states:
                    ws.make_placed_blocks_available()
                return next_world_states, None

            # ------------------------------ FIX and FIX ------------------------------
            if not world_state.is_left_hand_available():
                left_hand_action = RobotActions.FIX_LEFT_HAND
            if not world_state.is_right_hand_available():
                right_hand_action = RobotActions.FIX_RIGHT_HAND

            var_string = self._get_new_var_string(curr_var_string, var_num)
            if self.check_and_execute_action(world_state, move_num, var_string,
                                             left_hand_action, None, right_hand_action, None,
                                             next_world_states):
                for ws in next_world_states:
                    ws.make_placed_blocks_available()
                return next_world_states, None

            # ------------------------------ PLACE and FIX ------------------------------
            if not world_state.is_left_hand_available():
                left_hand_action = RobotActions.PLACE_LEFT_HAND
            if not world_state.is_right_hand_available():
                right_hand_action = RobotActions.FIX_RIGHT_HAND

            var_string = self._get_new_var_string(curr_var_string, var_num)
            if self.check_and_execute_action(world_state, move_num, var_string,
                                             left_hand_action, None, right_hand_action, None,
                                             next_world_states):
                for ws in next_world_states:
                    ws.make_placed_blocks_available()
                return next_world_states, None

            # ------------------------------ FIX and PLACE ------------------------------
            if not world_state.is_left_hand_available():
                left_hand_action = RobotActions.FIX_LEFT_HAND
            if not world_state.is_right_hand_available():
                right_hand_action = RobotActions.PLACE_RIGHT_HAND

            var_string = self._get_new_var_string(curr_var_string, var_num)
            if self.check_and_execute_action(world_state, move_num, var_string,
                                             left_hand_action, None, right_hand_action, None,
                                             next_world_states):
                for ws in next_world_states:
                    ws.make_placed_blocks_available()
                return next_world_states, None

            Msg.print_error("ERROR [_get_all_possible_next_actions]: something weird happened--cannot place/fix block")
            assert False

        # find all possible grabs
        var_num = var_num + 1
        if self._is_one_hand_only_plan:
            additional_curr_world_states, var_num = self._get_one_hand_only_grabs(world_state,
                                                                                  move_num=move_num,
                                                                                  parent_var_string=parent_var_string,
                                                                                  var_num=var_num)
        else:
            additional_curr_world_states, var_num = self._get_one_hand_only_grabs(world_state,
                                                                                  move_num=move_num,
                                                                                  parent_var_string=parent_var_string,
                                                                                  var_num=var_num)

            additional_curr_world_states.extend(self._get_two_hand_only_grabs(world_state,
                                                                              move_num=move_num,
                                                                              parent_var_string=parent_var_string,
                                                                              var_num=var_num)[0])
        return None, additional_curr_world_states

    @staticmethod
    def _get_pseudo_random_signs(total_signs, is_checked=False):
        if not is_checked:
            return [random.randint(0, 1) * 2 - 1 for _ in range(total_signs)]
        return_list = []
        prev_sign = -1
        prev_prev_sign = 1
        for _ in range(total_signs):
            new_sign = random.randint(0, 1) * 2 - 1
            if new_sign == prev_sign and new_sign == prev_prev_sign:
                new_sign = new_sign * -1

            prev_prev_sign = prev_sign
            prev_sign = new_sign

            return_list.append(new_sign)
        return return_list

    def _init_world_state_tree(self):
        self._root_world_state = WorldStateNode(self._init_blocks_list, move_num=0, variation_num=0)
        self._root_world_state.set_as_valid_to_explore()
        self._root_world_state.update_blocks_state()

        self._sign_by_height_x = self._get_pseudo_random_signs(self._max_steps_per_plan)
        self._sign_by_height_y = self._get_pseudo_random_signs(self._max_steps_per_plan)

    def _copy_fin_to_intermediate(self, check_if_exists=True):
        original_final_file_path = "fin_files/" + self._core_identifier + PathUtils.DOT_F_G
        inter_final_file_path = "intermediate_files/" + self._intermediate_folder_name + "/state" + PathUtils.DOT_F_G

        file_exists = FileUtils.is_file(PathUtils.join(PathUtils.get_library_dirpath(), inter_final_file_path))
        if (check_if_exists and not file_exists) or not check_if_exists:
            FileUtils.create_out_file(PathUtils.join(PathUtils.get_library_dirpath(), inter_final_file_path))
            copyfile(PathUtils.join(PathUtils.get_library_dirpath(), original_final_file_path),
                     PathUtils.join(PathUtils.get_library_dirpath(), inter_final_file_path))

    def check_and_execute_action(self, curr_world_state, query_move_num, query_var_num,
                                 robot_action_left, block_name_left,
                                 robot_action_right, block_name_right,
                                 valid_world_states_list, do_assignment=True):

        if do_assignment:
            if robot_action_left == RobotActions.GRAB and robot_action_right == RobotActions.GRAB:
                robot_hand1, robot_hand2 = curr_world_state.get_best_hands_for_two_grabs(block_name_left,
                                                                                         block_name_right)

                robot_action_left = RobotActions.GRAB_LEFT_HAND
                robot_action_right = RobotActions.GRAB_RIGHT_HAND
                if robot_hand1 == NameUtils.ROBOT_HAND_RIGHT:
                    robot_action_left = RobotActions.GRAB_RIGHT_HAND
                    robot_action_right = RobotActions.GRAB_LEFT_HAND

            elif robot_action_left == RobotActions.GRAB_LEFT_HAND:
                robot_hand1 = curr_world_state.get_best_hand_for_grab(block_name_left)

                if not robot_hand1 == NameUtils.ROBOT_HAND_LEFT:  # best hand is right hand
                    return False

            elif robot_action_right == RobotActions.GRAB_RIGHT_HAND:
                robot_hand1 = curr_world_state.get_best_hand_for_grab(block_name_right)

                if not robot_hand1 == NameUtils.ROBOT_HAND_RIGHT:  # best hand is right hand
                    return False

        query_world_state = self._get_new_world_state(world_state=curr_world_state,
                                                      move_num=query_move_num, variation_num=query_var_num)

        execute_status1 = query_world_state.execute_action(robot_action_left, block_name_left)
        execute_status2 = query_world_state.execute_action(robot_action_right, block_name_right)
        if execute_status1 and execute_status2 and query_world_state.are_actions_active():
            valid_world_states_list.append(query_world_state)
            return True

        # try another order of execution
        query_world_state = self._get_new_world_state(world_state=curr_world_state,
                                                      move_num=query_move_num, variation_num=query_var_num)

        execute_status1 = query_world_state.execute_action(robot_action_right, block_name_right)
        execute_status2 = query_world_state.execute_action(robot_action_left, block_name_left)
        if execute_status1 and execute_status2 and query_world_state.are_actions_active():
            valid_world_states_list.append(query_world_state)
            return True
        return False

    @staticmethod
    def _run_physx_executable(arguments_list):
        if ConfigUtils.DEBUG_PLANNER:
            return

        arguments_string = " ".join(arguments_list)

        Msg.print_info(arguments_string)

        if ConfigUtils.DEBUG_CPP:
            CPPUtils.run_physx(arguments=arguments_string, run_gdb=True)
        else:
            CPPUtils.run_physx(arguments=arguments_string, run_gdb=False)

    def _simulate(self, world_state: WorldStateNode,
                  noise_x_left=str(0.0), noise_y_left=str(0.0), noise_x_right=str(0.0), noise_y_right=str(0.0)):
        exp_name = self._core_identifier

        run_simulation_string = "0"
        if ConfigUtils.IS_PHYSICS_ON:
            run_simulation_string = "1"

        is_diff_string = "0"
        if self._is_diff:
            is_diff_string = "1"

        move_num = world_state.get_move_num()
        variation_string = world_state.get_variation_string()
        parent_variation_str = world_state.get_parent_variation_string()

        base_file_path = PathUtils.get_library_dirpath()
        out_file_path = "/intermediate_files/" + self._intermediate_folder_name + "/"
        FileUtils.create_dir(base_file_path + out_file_path)

        init_file_path = "/init_files/" + exp_name + PathUtils.DOT_G
        if move_num > 1:
            init_file_path = out_file_path + self._get_intermediate_state_filename(parent_variation_str)

        final_file_path = "/intermediate_files/" + self._intermediate_folder_name + "/state" + PathUtils.DOT_F_G

        og_fin_file_path = "/fin_files/" + exp_name + PathUtils.DOT_F_G

        block_names = "#".join(world_state.get_all_block_names())
        if self._is_diff and (self._exp_trial_num.startswith("17") or self._exp_trial_num.startswith("31")):
            block_names = "#".join([b for b in world_state.get_all_block_names() if b[0] == "b"])

        plan_as_string = world_state.get_action_details_as_string(delimiter=",").replace(" ", "_")
        grabbed_block_names_as_string = world_state.get_contents_as_string(delimiter=",")

        Msg.print_info("INFO: Running -- " + self._core_identifier + ", " + variation_string)

        self._run_physx_executable([run_simulation_string, base_file_path,
                                    init_file_path, final_file_path, og_fin_file_path, out_file_path, variation_string,
                                    block_names, plan_as_string, grabbed_block_names_as_string,
                                    noise_x_left, noise_y_left, noise_x_right, noise_y_right, is_diff_string])

        GFileUtils.clean_g_file(PathUtils.join(self._get_intermediate_core_dir_path(),
                                               self._get_intermediate_state_filename(variation_string)))

    def run_physx_simulation(self, world_state: WorldStateNode):
        def _is_sanity_check_successful(block_params_dict):
            for _, value in block_params_dict.items():
                position_values = value.get_position().get_values_as_list()
                for pos in position_values:
                    if np.isnan(pos):
                        return False
            return True

        def _get_random_truncated_normal():
            random_value = ComputeUtils.sample_trunc_normal(0.0,
                                                            -ConfigUtils.NOISE_GAUSSIAN_STD,
                                                            ConfigUtils.NOISE_GAUSSIAN_STD,
                                                            ConfigUtils.NOISE_GAUSSIAN_STD)
            return round(random_value, 5)

        left_hand_rec = world_state.get_left_hand_action_record()
        right_hand_rec = world_state.get_right_hand_action_record()
        left_is_fix = left_hand_rec.get_robot_action() == RobotActions.FIX_LEFT_HAND
        right_is_fix = right_hand_rec.get_robot_action() == RobotActions.FIX_RIGHT_HAND

        noise_x_left = 0.0
        noise_y_left = 0.0
        noise_x_right = 0.0
        noise_y_right = 0.0

        if ConfigUtils.IS_PHYSICS_ON:
            condition_left = left_is_fix and not left_hand_rec.get_fix_above_block_name() == NameUtils.TABLE
            condition_right = right_is_fix and not right_hand_rec.get_fix_above_block_name() == NameUtils.TABLE
            if condition_left:
                if self._is_diff:
                    block_height = world_state.get_block_height_by_block_name(left_hand_rec.get_block_name()) - 0.947
                    block_height_index = int(block_height / 0.051)
                    noise_x_left = self._sign_by_height_x[block_height_index] * abs(_get_random_truncated_normal())
                    noise_y_left = self._sign_by_height_y[block_height_index] * abs(_get_random_truncated_normal())
                else:
                    noise_x_left = 0.0
                    noise_y_left = self._get_pseudo_random_signs(1)[0] * abs(_get_random_truncated_normal())

            if condition_right:
                if self._is_diff:
                    block_height = world_state.get_block_height_by_block_name(right_hand_rec.get_block_name()) - 0.947
                    block_height_index = int(block_height / 0.051)
                    noise_x_right = self._sign_by_height_x[block_height_index] * abs(_get_random_truncated_normal())
                    noise_y_right = self._sign_by_height_y[block_height_index] * abs(_get_random_truncated_normal())
                else:
                    noise_x_right = 0.0
                    noise_y_right = self._get_pseudo_random_signs(1)[0] * abs(_get_random_truncated_normal())

        self._simulate(world_state,
                       noise_x_left=str(noise_x_left), noise_y_left=str(noise_y_left),
                       noise_x_right=str(noise_x_right), noise_y_right=str(noise_y_right))

        new_dict = self._get_new_blocks_params_from_intermediate_file(world_state.get_variation_string())

        if not _is_sanity_check_successful(new_dict):
            Msg.print_warn("WARN [run_physx_simulation]: found nan values in file")
            return False

        # update final block positions that take into account the noise added to the system while fixing blocks
        if left_is_fix:
            world_state.update_final_block_position(new_dict, left_hand_rec.get_block_name())
        if right_is_fix:
            world_state.update_final_block_position(new_dict, right_hand_rec.get_block_name())

        world_state.update_current_block_params(new_dict)

        return True

    def start_planner(self):
        def _get_all_combinations(list1, list2):
            if self._is_diff:
                return sorted(list(set([tuple(sorted(i)) for i in list(product(list1, list2))])),
                              key=lambda e: (int(e[0][1:]), int(e[1][1:])))
            return sorted(list(set([tuple(sorted(i)) for i in list(product(list1, list2))])))

        root_next_world_states = self.get_first_move_world_states()[::-1]
        for world_state in root_next_world_states:
            self._root_world_state.add_next_world_state(world_state)

        self._world_states_to_process = deque(root_next_world_states)
        # if self._is_diff:
        #     random.shuffle(self._world_states_to_process)
        #     for new_var_num, world_state in enumerate(self._world_states_to_process, 1):
        #         world_state.set_variation_string(str(new_var_num))

        self._copy_fin_to_intermediate(check_if_exists=False)

        while len(self._world_states_to_process) > 0:
            if self._is_diff:
                current_world_state = self._world_states_to_process.pop()
            else:
                current_world_state = self._world_states_to_process.popleft()

            current_world_state.update_blocks_state()

            prev_available_block_names = current_world_state.get_available_block_names_to_grab(self._is_diff)

            if not self.run_physx_simulation(current_world_state):
                continue

            if current_world_state.are_all_blocks_done():
                current_world_state.set_as_valid_to_explore()
                if self._is_diff:
                    break
                continue

            if current_world_state.get_move_num() >= self._max_steps_per_plan:
                if self._is_diff:
                    current_world_state.set_as_valid_to_explore()
                    break
                continue

            available_block_names = current_world_state.get_available_block_names_to_grab(self._is_diff)

            next_actions_list = sorted(current_world_state.get_next_moves_to_process(self._is_diff),
                                       key=lambda x: (x[0], x[1]))

            action_records_to_remove = []  # list of tuples of the kind: (left_hand_record, right_hand_record)
            next_valid_world_states = []

            if self._is_diff and len(prev_available_block_names) < len(available_block_names):  # restart planning
                self._world_states_to_process.clear()
                next_ws, additional_curr_ws = self._get_all_possible_re_plan_actions(current_world_state)
                if next_ws is not None:
                    next_valid_world_states.extend(next_ws)
                if additional_curr_ws is not None:
                    for ws_new in additional_curr_ws:
                        does_exist = False
                        for ws_old in self._world_states_to_process:
                            condition1 = ws_new.get_left_hand_action_record() == ws_old.get_left_hand_action_record()
                            condition2 = ws_new.get_right_hand_action_record() == ws_old.get_right_hand_action_record()
                            if condition1 and condition2:
                                does_exist = True
                                break
                        if does_exist:
                            continue
                        self._world_states_to_process.extend(additional_curr_ws)

            new_move_num = current_world_state.get_move_num() + 1
            current_var_str = current_world_state.get_variation_string()
            new_var_num = 1
            for action_left, action_right in next_actions_list:
                if action_left == action_right:  # precautionary
                    continue

                if RobotActions.are_both_inactive(action_left, action_right):
                    continue

                if self._is_one_hand_only_plan and not RobotActions.is_one_action_idle(action_left, action_right):
                    continue

                if self._is_diff:
                    condition1 = not self._is_one_hand_only_plan
                    condition2 = RobotActions.is_one_action_idle(action_left, action_right)
                    condition3 = len(available_block_names) > 1

                    if condition1 and condition2 and condition3:
                        continue

                if RobotActions.GRAB in action_left and RobotActions.GRAB in action_right:
                    if len(available_block_names) < 2:
                        continue

                    for block1_name, block2_name in _get_all_combinations(available_block_names, available_block_names):
                        if block1_name == block2_name:
                            continue

                        if self.check_and_execute_action(current_world_state,
                                                         new_move_num,
                                                         self._get_new_var_string(current_var_str, new_var_num),
                                                         RobotActions.GRAB, block1_name,
                                                         RobotActions.GRAB, block2_name,
                                                         next_valid_world_states):
                            action_records_to_remove.extend(self._get_moves_to_remove(next_valid_world_states[-1]))
                            new_var_num += 1

                elif RobotActions.GRAB in action_left:
                    if len(available_block_names) == 0:
                        continue

                    for block1_name in available_block_names:
                        if self.check_and_execute_action(current_world_state,
                                                         new_move_num,
                                                         self._get_new_var_string(current_var_str, new_var_num),
                                                         RobotActions.GRAB_LEFT_HAND, block1_name,
                                                         action_right, None,
                                                         next_valid_world_states):
                            action_records_to_remove.extend(self._get_moves_to_remove(next_valid_world_states[-1]))
                            new_var_num += 1

                elif RobotActions.GRAB in action_right:
                    if len(available_block_names) == 0:
                        continue

                    for block1_name in available_block_names:
                        if self.check_and_execute_action(current_world_state,
                                                         new_move_num,
                                                         self._get_new_var_string(current_var_str, new_var_num),
                                                         action_left, None,
                                                         RobotActions.GRAB_RIGHT_HAND, block1_name,
                                                         next_valid_world_states):
                            action_records_to_remove.extend(self._get_moves_to_remove(next_valid_world_states[-1]))
                            new_var_num += 1

                else:
                    if self.check_and_execute_action(current_world_state,
                                                     new_move_num,
                                                     self._get_new_var_string(current_var_str, new_var_num),
                                                     action_left, None,
                                                     action_right, None,
                                                     next_valid_world_states):
                        action_records_to_remove.extend(self._get_moves_to_remove(next_valid_world_states[-1]))
                        new_var_num += 1

            if len(next_valid_world_states) == 0:
                current_world_state.set_as_invalid_to_explore()
            else:
                next_world_states_exist = False
                new_var_num = 1

                next_valid_world_states = sorted(next_valid_world_states, reverse=True,
                                                 key=lambda x: x.get_sum_hand_content_at_final())

                for world_state in next_valid_world_states:
                    left_action_record = world_state.get_left_hand_action_record()
                    right_action_record = world_state.get_right_hand_action_record()
                    if (left_action_record, right_action_record) in action_records_to_remove:
                        continue

                    next_world_states_exist = True

                    # redo variation number
                    world_state.set_variation_string(self._get_new_var_string(current_var_str, new_var_num))
                    new_var_num += 1

                    current_world_state.add_next_world_state(world_state)
                    self._world_states_to_process.append(world_state)

                if next_world_states_exist:
                    current_world_state.set_as_valid_to_explore()
