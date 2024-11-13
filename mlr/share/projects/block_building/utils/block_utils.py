import numpy as np

from scipy.optimize import linear_sum_assignment

from mlr.share.projects.block_building.utils.compute_utils import ComputeUtils
from mlr.share.projects.block_building.utils.core_utils import NameUtils


class BlockPosition:
    def __init__(self, x=-9, y=-9, z=-9):
        self._x = x
        self._y = y
        self._z = z

    def x(self):
        return self._x

    def y(self):
        return self._y

    def z(self):
        return self._z

    def get_values_as_list(self):
        return [self.x(), self.y(), self.z()]

    def __eq__(self, other):
        if isinstance(other, BlockPosition):
            condition_x = ComputeUtils.is_close(self.x(), other.x(), 2)
            condition_y = ComputeUtils.is_close(self.y(), other.y(), 2)
            condition_z = ComputeUtils.is_close(self.z(), other.z(), 2)
            if condition_x and condition_y and condition_z:
                return True
        return False


class BlockQuaternion:
    def __init__(self, x=-1, y=-1, z=-1, w=-1):
        self._x = x
        self._y = y
        self._z = z
        self._w = w

    def x(self):
        return self._x

    def y(self):
        return self._y

    def z(self):
        return self._z

    def w(self):
        return self._w

    def get_values_as_list(self):
        return [self.x(), self.y(), self.z(), self.w()]

    def __eq__(self, other):
        if isinstance(other, BlockQuaternion):
            condition_x = ComputeUtils.is_close(self.x(), other.x())
            condition_y = ComputeUtils.is_close(self.y(), other.y())
            condition_z = ComputeUtils.is_close(self.z(), other.z())
            condition_w = ComputeUtils.is_close(self.w(), other.w())
            if condition_x and condition_y and condition_z and condition_w:
                return True
        return False


class BlockShape:
    def __init__(self, block_width=0.05, block_length=0.1, block_height=0.05, block_edge_curvature=0.005):
        self.block_width = float(block_width)
        self.block_length = float(block_length)
        self.block_height = float(block_height)
        self.block_edge_curvature = float(block_edge_curvature)

    def get_block_width(self):
        return self.block_width

    def get_block_length(self):
        return self.block_length

    def get_block_height(self):
        return self.block_height


class BlockParams:
    def __init__(self,
                 position: BlockPosition,
                 rotation: BlockQuaternion,
                 shape: BlockShape,
                 block_underneath_name,
                 block_above_name,
                 supports_two_blocks):

        self._position = position
        self._rotation = rotation
        self._shape = shape
        self._block_underneath_name = block_underneath_name
        self._block_above_name = block_above_name
        self._supports_two_blocks = supports_two_blocks

    def get_position(self):
        return self._position

    def get_shape(self):
        return self._shape

    def get_rotation(self):
        return self._rotation

    def get_rotation_as_list(self):
        return self._rotation.get_values_as_list()

    def get_name_of_block_underneath(self):
        return self._block_underneath_name

    def get_name_of_block_above(self):
        return self._block_above_name

    def set_position(self, position):
        self._position = position

    def set_rotation(self, rotation):
        self._rotation = rotation

    def set_name_of_block_underneath(self, name_of_block_underneath):  # can be "table"
        self._block_underneath_name = name_of_block_underneath

    def set_name_of_block_above(self, is_below_name):
        self._block_above_name = is_below_name

    def supports_two_blocks(self):
        return self._supports_two_blocks

    @staticmethod
    def get_hungarian_algorithm_fits(block_params1_dict, block_params2_dict):
        def _extract_position_info_as_list(input_dict, block_type):
            return_dict = {}
            for key in input_dict.keys():
                if key.startswith(block_type):
                    return_dict[key] = input_dict[key].get_position().get_values_as_list()
            return return_dict

        def _compute_l2_dist(coord_1, coord_2):
            x_diff = (coord_1[0] - coord_2[0]) ** 2
            y_diff = (coord_1[1] - coord_2[1]) ** 2
            z_diff = (coord_1[2] - coord_2[2]) ** 2
            return np.sqrt(x_diff + y_diff + z_diff)

        init_b_blocks = _extract_position_info_as_list(block_params1_dict, NameUtils.NAME_DIFF_B)
        init_c_blocks = _extract_position_info_as_list(block_params1_dict, NameUtils.NAME_DIFF_C)

        fin_b_blocks = _extract_position_info_as_list(block_params2_dict, NameUtils.NAME_DIFF_B)
        fin_c_blocks = _extract_position_info_as_list(block_params2_dict, NameUtils.NAME_DIFF_C)

        sorted_reference_b = [NameUtils.NAME_DIFF_B + str(num) for num in range(22)]
        sorted_reference_c = [NameUtils.NAME_DIFF_C + str(num) for num in range(17)]

        # Sort each corresponding pair for the matrix in the Hungarian Algorithm
        sorted_init_b_block_list = sorted(init_b_blocks.keys(), key=lambda x: sorted_reference_b.index(x))
        sorted_fin_b_block_list = sorted(fin_b_blocks.keys(), key=lambda x: sorted_reference_b.index(x))

        sorted_init_c_block_list = sorted(init_c_blocks.keys(), key=lambda x: sorted_reference_c.index(x))
        sorted_fin_c_block_list = sorted(fin_c_blocks.keys(), key=lambda x: sorted_reference_c.index(x))

        # Run Hungarian Algorithm for b blocks and get assignment matrix
        b_dist_matrix = np.zeros((len(sorted_init_b_block_list), len(sorted_fin_b_block_list)))

        for i in range(len(sorted_init_b_block_list)):
            for j in range(len(sorted_fin_b_block_list)):
                b_dist_matrix[i][j] = _compute_l2_dist(init_b_blocks[sorted_fin_b_block_list[i]],
                                                       fin_b_blocks[sorted_fin_b_block_list[j]])

        assignments_b = linear_sum_assignment(b_dist_matrix)

        # Run Hungarian Algorithm for c blocks and get assignment matrix
        c_dist_matrix = np.zeros((len(sorted_init_c_block_list), len(sorted_fin_c_block_list)))

        for i in range(len(sorted_init_c_block_list)):
            for j in range(len(sorted_fin_c_block_list)):
                c_dist_matrix[i][j] = _compute_l2_dist(init_c_blocks[sorted_fin_c_block_list[i]],
                                                       fin_c_blocks[sorted_fin_c_block_list[j]])

        assignments_c = linear_sum_assignment(c_dist_matrix)

        return assignments_b, assignments_c

    def __eq__(self, other):
        if isinstance(other, BlockParams):
            condition2 = self.get_position() == other.get_position()
            condition3 = self.get_rotation() == other.get_rotation()
            condition1 = self.get_name_of_block_underneath() == other.get_name_of_block_underneath()
            condition4 = self.get_name_of_block_above() == other.get_name_of_block_above()
            if condition1 and condition2 and condition3 and condition4:
                return True
        return False
