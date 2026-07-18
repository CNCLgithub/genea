import numpy as np
import scipy.stats as stats

from collections.abc import Iterable
from scipy.spatial.transform import Rotation


class ComputeUtils:
    @staticmethod
    def flatten(input_list):
        flattened_list = []

        for item in input_list:
            if isinstance(item, Iterable) and not isinstance(item, (str, bytes)):
                flattened_list.extend(ComputeUtils.flatten(item))
            else:
                flattened_list.append(item)

        return flattened_list

    @staticmethod
    def sample_uniform(min_value, max_value, size=1):
        return np.atleast_1d(np.random.uniform(min_value, max_value, size))

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
        return distribution.ppf(ComputeUtils.sample_uniform(cdf_min, cdf_max).item())

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

    @staticmethod
    def compute_paired_t_test(list1, list2):
        return stats.ttest_rel(list1, list2)

    @staticmethod
    def compute_t_test(list1, list2):
        return stats.ttest_ind(list1, list2)
