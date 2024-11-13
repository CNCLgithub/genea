import numpy as np
import scipy.stats as stats

from scipy.spatial.transform import Rotation


class ComputeUtils:
    @staticmethod
    def is_close(float1, float2, threshold_decimal_places=2):
        """
        @param float1                   : the first number to compare
        @param float2                   : the second number to compare
        @param threshold_decimal_places : the number of positions after the decimal point beyond which values are same
        """
        return np.isclose([float1], [float2], atol=10 ** -threshold_decimal_places)[0]

    @staticmethod
    def get_euler_angles(quaternion, existing_quaternions_dict=None):
        """
        returns : an array of 3 numbers where --
                    First angle belongs to [-180, 180] degrees
                    Second angle belongs to [-90, 90] degrees
                    Third angle belongs to [-180, 180] degrees
        """
        if existing_quaternions_dict is None:
            existing_quaternions_dict = {}

        key = tuple(quaternion)
        if key in existing_quaternions_dict:
            return existing_quaternions_dict[key]

        if quaternion.count(quaternion[0]) == len(quaternion):
            return -1, -1, -1

        rot = Rotation.from_quat(quaternion)
        rot_euler = rot.as_euler('xyz', degrees=True)

        rot_euler = [round(angle, 2) for angle in rot_euler]

        existing_quaternions_dict[key] = rot_euler

        return rot_euler

    @staticmethod
    def sample_trunc_normal(mu, bound_min, bound_max, sigma=0.5):
        lower = (bound_min - mu) / sigma
        upper = (bound_max - mu) / sigma
        return stats.truncnorm.rvs(lower, upper, loc=mu, scale=sigma, size=1).tolist()[0]

    @staticmethod
    def zscore_list(input_list):
        return list(stats.zscore(input_list))
