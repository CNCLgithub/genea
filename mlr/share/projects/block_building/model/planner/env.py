import numpy as np

from collections import OrderedDict
from shapely.geometry import Polygon

from mlr.share.projects.block_building.utils.block_utils import BlockParams
from mlr.share.projects.block_building.utils.core_utils import ConfigUtils
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
