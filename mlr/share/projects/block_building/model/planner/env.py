import math
from collections import OrderedDict

from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.msg_utils import Msg


class Table:
    class _OccupiedSpots:
        def __init__(self, min_x=math.inf, max_x=math.inf, min_y=math.inf, max_y=math.inf):
            self.min_x = round(min_x, 3)
            self.max_x = round(max_x, 3)

            self.min_y = round(min_y, 3)
            self.max_y = round(max_y, 3)

        def is_within(self, x_min, x_max, y_min, y_max):
            dx = min(self.max_x, x_max) - max(self.min_x, x_min)
            dy = min(self.max_y, y_max) - max(self.min_y, y_min)
            if dx >= 0 and dy >= 0:
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

    def is_spot_free(self, pos_min_x, pos_max_x, pos_min_y, pos_max_y):
        for block_name, value in self._occupied_spots_dict.items():
            if value.is_within(pos_min_x, pos_max_x, pos_min_y, pos_max_y):
                return False
        return True

    def update_occupied_spots_list(self, block_list):
        self.is_left_place_taken = False
        self.is_right_place_taken = False

        for block in block_list:
            if block.is_placed_left() or block.is_placed_left_unavailable():
                self.is_left_place_taken = True
            elif block.is_placed_right() or block.is_placed_right_unavailable():
                self.is_right_place_taken = True

            # invalidate previously held position for blocks that are grabbed or placed
            if block.is_grabbed() or block.is_placed() or block.is_placed_but_unavailable():
                self._occupied_spots_dict[block.get_block_name()] = self._OccupiedSpots()
                continue

            if block.get_current_pos() is None:
                Msg.print_error("ERROR [update_occupied_spots_list]: block position unavailable in unexpected state")
                assert False

            pos_x = block.get_current_pos().x()
            pos_y = block.get_current_pos().y()

            block_width = block.get_shape().get_block_width()
            block_length = block.get_shape().get_block_length()
            block_height = block.get_shape().get_block_height()

            angles = ComputeUtils.get_euler_angles(block.get_current_rot().get_values_as_list())

            cos_angle = round(math.cos(math.radians(abs(angles[2]))), 2)

            multiplier = 0.68

            rectilinear_angles = [0, 90, 180, 270, 360]
            if any([rect_angle - 4 <= angles[1] <= rect_angle + 4 for rect_angle in rectilinear_angles]):
                multiplier = 0.51

            min_x = pos_x - multiplier * abs(block_width)
            max_x = pos_x + multiplier * abs(block_width)

            min_y = pos_y - multiplier * cos_angle * abs(block_length)
            max_y = pos_y + multiplier * cos_angle * abs(block_length)
            if ComputeUtils.is_close(cos_angle, 0.0):  # long block is vertical
                min_y = pos_y - multiplier * abs(block_height)
                max_y = pos_y + multiplier * abs(block_height)

            self._occupied_spots_dict[block.get_block_name()] = self._OccupiedSpots(min_x, max_x, min_y, max_y)
