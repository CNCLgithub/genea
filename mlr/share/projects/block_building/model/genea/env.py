import numpy as np

from collections import OrderedDict
from shapely.geometry import Polygon

from mlr.share.projects.block_building.utils.block_utils import BlockParams, BlockPosition, BlockQuaternion
from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.core_utils import ConfigUtils, NameUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg
from mlr.share.projects.block_building.utils.plot_utils import PlotUtils


class Table:
    class _OccupiedSpots:
        def __init__(self, block_vertices=None, color='blue'):
            self._block_vertices = block_vertices

            colors_list = {
                "blue": "#008cff",
                "red": "#e53333",
                "emerald": "#00d900",
                "mauve": "#9969d9",
                "yellow": "#e5e500",
                "black": "#000000"
            }
            colors_list.update({f"b{i}": "#000000" for i in range(0, 35)})
            colors_list.update({f"c{i}": "#e66600" for i in range(0, 35)})
            self.color = colors_list[color]

        def get_vertices(self):
            return self._block_vertices

        def get_color(self):
            return self.color

        def block_exists(self):
            if self.get_vertices() is None:
                return False
            return True

        def is_within(self, query_block_vertices):
            if not self.block_exists():
                return False

            intersection = Polygon(self.get_vertices()).intersection(Polygon(query_block_vertices))

            if intersection.area > 0.0005:  # 20% of the area of the smallest piece
                return True
            return False

    def __init__(self, init_blocks_list):
        """
        positions_occupied: a list of tuples [(ya1,ya2), (yb1, yb2), ...] that represent where the table has blocks
        """
        self._occupied_spots_dict = OrderedDict()
        for block in init_blocks_list:
            self._occupied_spots_dict[block.get_block_name()] = self._OccupiedSpots()

        self.is_left_place_taken = False
        self.is_right_place_taken = False

        self.update_occupied_spots_list(init_blocks_list)

    def is_place_left_available(self):
        return not self.is_left_place_taken

    def is_place_right_available(self):
        return not self.is_right_place_taken

    def is_spot_free(self, query_block):
        if query_block.get_shape().get_block_height() == 0.25:  # exception for diff_12_1
            return True
        
        query_block_vertices = BlockParams.get_block_vertices(query_block, at_final=True)
        for block_name, block_occupied_spot in self._occupied_spots_dict.items():
            if block_occupied_spot.is_within(query_block_vertices):
                if ConfigUtils.DEBUG_PLANNER:
                    self.visualize_table()
                return False
        return True

    def update_occupied_spots_list(self, block_list):
        self.is_left_place_taken = False
        self.is_right_place_taken = False

        for block in block_list:
            block_name = block.get_block_name()

            if block.is_placed_left() or block.is_placed_left_unavailable():
                self.is_left_place_taken = True
            elif block.is_placed_right() or block.is_placed_right_unavailable():
                self.is_right_place_taken = True

            # invalidate previously held position for blocks that are grabbed or placed
            if block.is_grabbed() or block.is_placed() or block.is_placed_but_unavailable():
                self._occupied_spots_dict[block_name] = self._OccupiedSpots()
                continue

            if block.get_current_pos() is None:
                Msg.print_error("ERROR [update_occupied_spots_list]: block position unavailable in unexpected state")
                assert False

            block_vertices = BlockParams.get_block_vertices(block)

            self._occupied_spots_dict[block_name] = self._OccupiedSpots(block_vertices, block_name)

    def visualize_table(self):
        """
        This shows the visualization of the table from the other side of the agent
        """
        rot_for_vis = np.array([[0, -1], [1, 0],]).T

        block_vertices_list = []
        for _, spot in self._occupied_spots_dict.items():
            if spot.block_exists():
                block_vertices_list.append(spot.get_vertices() @ rot_for_vis)

        colors_list = [spot.get_color() for _, spot in self._occupied_spots_dict.items()]
        names_list = [b_name for b_name, _ in self._occupied_spots_dict.items()]

        is_diff = all(s.startswith(('b', 'c')) for s in list(self._occupied_spots_dict.keys()))
        PlotUtils.draw_blocks_table_plot(block_vertices_list, colors_list, names_list, is_diff)


class BlockState:
    GRABBED_LEFT = "GRABBED_LEFT"
    GRABBED_RIGHT = "GRABBED_RIGHT"

    PLACED_LEFT = "PLACED_LEFT"
    PLACED_RIGHT = "PLACED_RIGHT"

    # state that the block assumes immediately after it is placed
    PLACED_LEFT_UNAVAILABLE_ONE = "PLACED_LEFT_UNAVAILABLE_ONE"
    PLACED_LEFT_UNAVAILABLE_TWO = "PLACED_LEFT_UNAVAILABLE_TWO"
    PLACED_RIGHT_UNAVAILABLE_ONE = "PLACED_RIGHT_UNAVAILABLE_ONE"
    PLACED_RIGHT_UNAVAILABLE_TWO = "PLACED_RIGHT_UNAVAILABLE_TWO"

    AVAILABLE = "AVAILABLE"
    DONE = "DONE"  # when the block is in its final configuration

    UNASSIGNED = "UNASSIGNED"

    def __init__(self):
        self.state = self.UNASSIGNED

    def is_grabbed_left(self):
        return self.state == self.GRABBED_LEFT

    def is_grabbed_right(self):
        return self.state == self.GRABBED_RIGHT

    def is_placed_left(self):
        return self.state == self.PLACED_LEFT

    def is_placed_right(self):
        return self.state == self.PLACED_RIGHT

    def is_placed_left_unavailable_one(self):
        return self.state == self.PLACED_LEFT_UNAVAILABLE_ONE

    def is_placed_left_unavailable_two(self):
        return self.state == self.PLACED_LEFT_UNAVAILABLE_TWO

    def is_placed_right_unavailable_one(self):
        return self.state == self.PLACED_RIGHT_UNAVAILABLE_ONE

    def is_placed_right_unavailable_two(self):
        return self.state == self.PLACED_RIGHT_UNAVAILABLE_TWO

    def is_available(self):
        return self.state == self.AVAILABLE

    def is_unassigned(self):
        return self.state == self.UNASSIGNED

    def is_done(self):
        return self.state == self.DONE

    def update_state(self, new_state):
        self.state = new_state


class Block:
    def __init__(self, block_name, init_block_params: BlockParams, final_block_params: BlockParams, is_diff):
        self.block_name = block_name  # name of the block as is mentioned in the GFile

        self._current_block_params = init_block_params
        self._final_block_params = final_block_params

        self._is_diff = is_diff

        self._block_state = BlockState()

        self._has_been_placed = False
        self._cached_quaternion_to_degree_dict = {}

    # ------------------------------- GETTERS -------------------------------

    def get_block_name(self):
        return self.block_name

    def get_name_of_block_underneath(self):
        if self._current_block_params is None:
            Msg.print_error("ERROR [get_name_of_block_underneath]: invalid operation")
            return None
        return self._current_block_params.get_name_of_block_underneath()

    def get_name_of_block_underneath_at_final(self):
        return self._final_block_params.get_name_of_block_underneath()

    def get_name_of_block_above(self):
        if self._current_block_params is None:
            Msg.print_error("ERROR [get_name_of_block_above]: invalid operation")
            return None
        return self._current_block_params.get_name_of_block_above()

    def get_name_of_block_above_at_final(self):
        return self._final_block_params.get_name_of_block_above()

    def get_current_block_params(self):
        return self._current_block_params

    def get_current_pos(self) -> BlockPosition:
        if self._current_block_params.get_position() is None:
            Msg.print_error("ERROR [get_current_pos]: invalid operation")
            assert False
        return self._current_block_params.get_position()

    def get_final_pos(self) -> BlockPosition:
        return self._final_block_params.get_position()

    def get_current_rot(self) -> BlockQuaternion:
        if self._current_block_params.get_rotation() is None:
            Msg.print_error("ERROR [get_current_rot_as_quaternion]: invalid operation")
            assert False
        return self._current_block_params.get_rotation()

    def get_final_rot(self) -> BlockQuaternion:
        return self._final_block_params.get_rotation()

    def get_shape(self):
        return self._current_block_params.get_shape()

    def get_num_of_blocks_on_top(self):  # assumes that only one block can be placed on top of another
        if self.is_free_to_support():
            return 0

        if self.is_grabbed() or self.is_placed() or self.is_placed_but_unavailable():
            return 0

        return 1

    # ------------------------------- SETTERS -------------------------------

    def set_name_of_block_above(self, block_on_top_name):
        if self._current_block_params is not None:
            self._current_block_params.set_name_of_block_above(block_on_top_name)
            return

        Msg.print_error("ERROR [set_is_supporting_another_block]: attempted to set support for unavailable block")

    # ------------------------------- CHECKERS -------------------------------

    def is_block_at_final_pos(self):
        def is_in_range(block1_params: BlockParams, block2_params: BlockParams, cached_dict: dict):

            angles1 = ComputeUtils.get_euler_angles(block1_params.get_rotation_as_list(), cached_dict)
            angles2 = ComputeUtils.get_euler_angles(block2_params.get_rotation_as_list(), cached_dict)
            for detail1, detail2 in zip(angles1, angles2):
                if not 0.0 <= abs(detail1 - detail2) <= 8.0 and not 352.0 <= abs(detail1 - detail2) <= 360.0:
                    return False

            if not block1_params.get_position() == block2_params.get_position():
                if self._is_diff:
                    b1 = block1_params.get_position().get_values_as_list()
                    b2 = block2_params.get_position().get_values_as_list()
                    for detail1, detail2 in zip(b1, b2):
                        if abs(detail1 - detail2) > ConfigUtils.NOISE_THRESHOLD:
                            return False
                    return True
                return False
            return True

        if self._block_state.is_done():
            return True

        if self.is_grabbed() or self.is_placed() or self.is_placed_but_unavailable():
            return False

        block_underneath_current = self.get_name_of_block_underneath()
        block_underneath_final = self.get_name_of_block_underneath_at_final()

        if self._is_diff and not block_underneath_current == block_underneath_final:
            return False

        if is_in_range(self._current_block_params, self._final_block_params, self._cached_quaternion_to_degree_dict):
            self._block_state.update_state(BlockState.DONE)
            return True
        return False

    def is_grabbed_left_hand(self):
        return self._block_state.is_grabbed_left()

    def is_grabbed_right_hand(self):
        return self._block_state.is_grabbed_right()

    def is_grabbed(self):
        return self.is_grabbed_left_hand() or self.is_grabbed_right_hand()

    def is_placed_left(self):
        return self._block_state.is_placed_left()

    def is_placed_right(self):
        return self._block_state.is_placed_right()

    def is_placed(self):
        return self.is_placed_left() or self.is_placed_right()

    def _is_placed_left_unavailable_one(self):
        return self._block_state.is_placed_left_unavailable_one()

    def _is_placed_left_unavailable_two(self):
        return self._block_state.is_placed_left_unavailable_two()

    def _is_placed_right_unavailable_one(self):
        return self._block_state.is_placed_right_unavailable_one()

    def _is_placed_right_unavailable_two(self):
        return self._block_state.is_placed_right_unavailable_two()

    def is_placed_left_unavailable(self):
        return self._is_placed_left_unavailable_one() or self._is_placed_left_unavailable_two()

    def is_placed_right_unavailable(self):
        return self._is_placed_right_unavailable_one() or self._is_placed_right_unavailable_two()

    def is_placed_but_unavailable(self):
        return self.is_placed_left_unavailable() or self.is_placed_right_unavailable()

    def is_free_to_support(self):
        if self.is_grabbed() or self.is_placed() or self.is_placed_but_unavailable():
            return False

        if self._current_block_params.get_name_of_block_above() == NameUtils.NO_BLOCK_ON_TOP:
            return True

        if self._final_block_params.supports_two_blocks():
            return True

        return False

    def is_available(self):
        return self._block_state.is_available()

    def has_been_placed(self):
        if self._is_diff:  # a block can be placed multiple times in a run given the nature of the experiment
            return False
        return self._has_been_placed

    # ------------------------------- OTHER FUNCTIONS -------------------------------

    def update_current_block_params(self, new_block_params):
        self._current_block_params = new_block_params
        if self._block_state.is_done():
            self._block_state.update_state(BlockState.UNASSIGNED)
            if self.is_block_at_final_pos():
                self._block_state.update_state(BlockState.DONE)
            else:
                self._block_state.update_state(BlockState.AVAILABLE)
        else:
            if self.is_block_at_final_pos():
                self._block_state.update_state(BlockState.DONE)

    def update_final_block_position(self, new_final_block_position):
        new_position = BlockPosition(new_final_block_position.x(),
                                     new_final_block_position.y(),
                                     self._final_block_params.get_position().z())
        self._final_block_params.set_position(new_position)

    def make_block_available(self):
        if self._is_placed_left_unavailable_one():
            self._block_state.update_state(BlockState.PLACED_LEFT_UNAVAILABLE_TWO)
            return
        elif self._is_placed_left_unavailable_two():
            self._block_state.update_state(BlockState.PLACED_LEFT)
            return
        elif self._is_placed_right_unavailable_one():
            self._block_state.update_state(BlockState.PLACED_RIGHT_UNAVAILABLE_TWO)
            return
        elif self._is_placed_right_unavailable_two():
            self._block_state.update_state(BlockState.PLACED_RIGHT)
            return
        elif self._block_state.is_unassigned():
            self._block_state.update_state(BlockState.AVAILABLE)
        elif self._block_state.is_done():
            Msg.print_warn("assigning AVAILABLE to a block that's DONE")
            self._block_state.update_state(BlockState.AVAILABLE)

    def move_to_final_pos(self):
        self._current_block_params = self._final_block_params
        self._current_block_params.set_name_of_block_above(NameUtils.NO_BLOCK_ON_TOP)
        self._block_state.update_state(BlockState.DONE)

    def move_to_left_hand(self):
        self._current_block_params.set_position(None)
        self._current_block_params.set_rotation(None)
        self._current_block_params.set_name_of_block_above(NameUtils.NO_BLOCK_ON_TOP)
        self._current_block_params.set_name_of_block_underneath(NameUtils.NO_BLOCK_UNDERNEATH)
        self._block_state.update_state(BlockState.GRABBED_LEFT)

    def move_to_right_hand(self):
        self._current_block_params.set_position(None)
        self._current_block_params.set_rotation(None)
        self._current_block_params.set_name_of_block_above(NameUtils.NO_BLOCK_ON_TOP)
        self._current_block_params.set_name_of_block_underneath(NameUtils.NO_BLOCK_UNDERNEATH)
        self._block_state.update_state(BlockState.GRABBED_RIGHT)

    def move_to_place_left(self):
        self._has_been_placed = True
        self._current_block_params.set_position(None)
        self._current_block_params.set_rotation(None)
        self._current_block_params.set_name_of_block_above(NameUtils.NO_BLOCK_ON_TOP)
        self._current_block_params.set_name_of_block_underneath(NameUtils.TABLE_PLACE_LEFT)
        self._block_state.update_state(BlockState.PLACED_LEFT_UNAVAILABLE_ONE)

    def move_to_place_right(self):
        self._has_been_placed = True
        self._current_block_params.set_position(None)
        self._current_block_params.set_rotation(None)
        self._current_block_params.set_name_of_block_above(NameUtils.NO_BLOCK_ON_TOP)
        self._current_block_params.set_name_of_block_underneath(NameUtils.TABLE_PLACE_RIGHT)
        self._block_state.update_state(BlockState.PLACED_RIGHT_UNAVAILABLE_ONE)
