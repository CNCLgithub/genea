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
    def sample_uniform(min_value, max_value, size=1):
        return np.asarray(np.random.uniform(min_value, max_value, size)).tolist()

    @staticmethod
    def sample_normal(mu, sigma):
        return np.random.normal(mu, sigma)

    @staticmethod
    def sample_trunc_normal(mu, bound_min, bound_max, sigma=1.0):
        if bound_min > bound_max:
            bound_min, bound_max = bound_max, bound_min

        lower = (bound_min - mu) / sigma
        upper = (bound_max - mu) / sigma
        return stats.truncnorm.rvs(lower, upper, loc=mu, scale=sigma, size=1).item()

    @staticmethod
    def sample_skew_normal(mu, sigma, alpha, bound_min, bound_max):
        """
        NOTE:

        alpha > 0 → right-skewed
        alpha < 0 → left-skewed
        alpha = 0 → normal distribution
        """
        if bound_min > bound_max:
            bound_min, bound_max = bound_max, bound_min

        distribution = stats.skewnorm(alpha, loc=mu, scale=sigma)
        cdf_min = distribution.cdf(bound_min)
        cdf_max = distribution.cdf(bound_max)
        return distribution.ppf(stats.uniform.rvs(cdf_min, cdf_max - cdf_min, size=1).item())

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
        return np.asarray(stats.zscore(input_list)).tolist()

    @staticmethod
    def get_spearmanr(list1, list2, with_p_value=False):
        """
        returns the spearman r and p-value as a list
        """
        if with_p_value:
            return stats.spearmanr(list1, list2)
        return stats.spearmanr(list1, list2)[0]

    @staticmethod
    def get_pearsonr(list1, list2, with_p_value=False):
        """
        returns the pearson r and p-value as a list
        """
        if with_p_value:
            return stats.pearsonr(list1, list2)
        return stats.pearsonr(list1, list2)[0]

    @staticmethod
    def convert_quat_to_euler(quat_xyzw):
        return Rotation.from_quat(quat_xyzw).as_euler('xyz')

    @staticmethod
    def compute_l2_distance(list1, list2):
        return np.linalg.norm(np.array(list1) - np.array(list2))

    @staticmethod
    def compute_angle_magnitude(rpy1, rpy2):
        r1 = Rotation.from_euler('xyz', rpy1)
        r2 = Rotation.from_euler('xyz', rpy2)

        relative_rot = r1.inv() * r2
        return np.degrees(relative_rot.magnitude())
