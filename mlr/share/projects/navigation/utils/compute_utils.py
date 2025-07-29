import numpy as np
import scipy.stats as stats

from itertools import combinations
from scipy.spatial.transform import Rotation


class ComputeUtils:
    @staticmethod
    def get_unique_combinations(input_list, length_of_combinations):
        """
        returns a list of indices, each equal to the length_of_combinations specified (e.g., 2 for a pair of indices)
        """
        return list(combinations(range(len(input_list)), length_of_combinations))

    @staticmethod
    def sample_trunc_normal(mu, bound_min, bound_max, sigma=1.0):
        lower = (bound_min - mu) / sigma
        upper = (bound_max - mu) / sigma
        return stats.truncnorm.rvs(lower, upper, loc=mu, scale=sigma, size=1).tolist()[0]

    @staticmethod
    def sample_trunc_normal_circular(center, bound_radius, sigma=1.0):
        while True:
            x = ComputeUtils.sample_trunc_normal(center, center - bound_radius, center + bound_radius, sigma)
            y = ComputeUtils.sample_trunc_normal(center, center - bound_radius, center + bound_radius, sigma)
            if (x - center) ** 2 + (y - center) ** 2 <= bound_radius ** 2:
                return x, y

    @staticmethod
    def chunkify_list(input_list, total_chunks):
        k, m = divmod(len(input_list), total_chunks)
        return list(input_list[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(total_chunks))

    @staticmethod
    def zscore_list(input_list):
        return list(stats.zscore(input_list))

    @staticmethod
    def compute_l2_distance(list1, list2):
        return np.linalg.norm(np.array(list1) - np.array(list2))

    @staticmethod
    def compute_angle_magnitude(rpy1, rpy2):
        r1 = Rotation.from_euler('xyz', rpy1)
        r2 = Rotation.from_euler('xyz', rpy2)

        relative_rot = r1.inv() * r2
        return np.degrees(relative_rot.magnitude())
