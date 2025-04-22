import numpy as np


class NavPosition:
    def __init__(self, x, y, z):
        self._pos_x = x
        self._pos_y = y
        self._pos_z = z

    def get_position_as_list(self):
        return [self._pos_x, self._pos_y, self._pos_z]

    def get_position_as_np_array(self):
        return np.array(self.get_position_as_list())


class NavRotation:
    def __init__(self, roll, pitch, yaw):
        self._rot_roll = roll
        self._rot_pitch = pitch
        self._rot_yaw = yaw

    def get_rotation_as_list(self):
        return [self._rot_roll, self._rot_pitch, self._rot_yaw]

    def get_rotation_as_np_array(self):
        return np.array(self.get_rotation_as_list())


class NavPose:
    def __init__(self, position: NavPosition, rotation: NavRotation):
        self._position = position
        self._rotation = rotation

    def get_position(self):
        return self._position

    def get_rotation(self):
        return self._rotation


class NavForce:
    def __init__(self, force_pose: NavPose, force_magnitude):
        self._force_magnitude = force_magnitude
        self._force_pose = force_pose

    def get_force_magnitude(self):
        return self._force_magnitude

    def get_force_pose(self):
        return self._force_pose
