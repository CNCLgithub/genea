import numpy as np


class NavPosition:
    def __init__(self, x, y, z):
        self._pos_x = x
        self._pos_y = y
        self._pos_z = z

    def add_x(self, x):
        self._pos_x += x

    def add_y(self, y):
        self._pos_y += y

    def add_z(self, z):
        self._pos_z += z

    def set_x(self, new_x):
        self._pos_x = new_x

    def get_x(self):
        return self._pos_x

    def get_y(self):
        return self._pos_y

    def get_z(self):
        return self._pos_z

    def get_position_as_list(self):
        return [self._pos_x, self._pos_y, self._pos_z]

    def get_position_as_np_array(self):
        return np.array(self.get_position_as_list())

    def get_position_as_str(self):
        return f"{self._pos_x} {self._pos_y} {self._pos_z}"


class NavRotation:
    def __init__(self, roll, pitch, yaw):
        self._rot_roll = roll
        self._rot_pitch = pitch
        self._rot_yaw = yaw

    def get_rotation_as_list(self):
        return [self._rot_roll, self._rot_pitch, self._rot_yaw]

    def get_rotation_as_np_array(self):
        return np.array(self.get_rotation_as_list())

    def get_rotation_as_str(self):
        return f"{self._rot_roll} {self._rot_pitch} {self._rot_yaw}"


class NavPose:
    def __init__(self, position: NavPosition, rotation: NavRotation = NavRotation(0.0, 0.0, 0.0)):
        self._position = position
        self._rotation = rotation

    @staticmethod
    def neutral():
        return NavPose(NavPosition(0.0, 0.0, 0.0))

    def set_rpy(self, roll, pitch, yaw):
        self._rotation = NavRotation(roll, pitch, yaw)

    def get_position(self):
        return self._position

    def get_rotation(self):
        return self._rotation


class NavForce:
    def __init__(self, force_pose: NavPose, force_magnitude):
        self._force_magnitude = force_magnitude
        self._force_pose = force_pose

    def set_force_magnitude(self, force_magnitude):
        self._force_magnitude = force_magnitude

    def get_force_magnitude(self):
        return self._force_magnitude

    def get_force_pose(self):
        return self._force_pose
