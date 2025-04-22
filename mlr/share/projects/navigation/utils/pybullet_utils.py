import numpy as np
import pybullet
import pybullet_data
import time

from mlr.share.projects.navigation.utils.config_utils import NavConfigUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavForce, NavPosition, NavRotation


class PyBulletRobot:
    def __init__(self, pybullet_id, pin_model):
        self.nq = pin_model.nq
        self.nv = pin_model.nv
        self.robot_id = pybullet_id
        self.pin_model = pin_model

        bullet_joint_map = {}
        for joint in range(pybullet.getNumJoints(self.robot_id)):
            bullet_joint_map[pybullet.getJointInfo(self.robot_id, joint)[1].decode("UTF-8")] = joint


class PyBulletUtils:
    def __init__(self, with_gui=False):
        self._platforms = []

        self._video_filepath = None

        if with_gui:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=NavConfigUtils.SIMULATION_TIME, numSubSteps=1)
        pybullet.setTimeStep(NavConfigUtils.SIMULATION_TIME)

    @staticmethod
    def add_ground_plane(ground_pos=NavPosition(0.0, 0.0, -NavConfigUtils.PLATFORM_HEIGHT)):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pos_as_list = ground_pos.get_position_as_list()
        pybullet.loadURDF("plane.urdf", pos_as_list, useFixedBase=True)

    def add_object_from_urdf(self, urdf_dirpath, urdf_filepath, pos: NavPosition, rot: NavRotation):
        pybullet.setAdditionalSearchPath(urdf_dirpath)
        pos_as_list = pos.get_position_as_list()
        rot_as_quat = pybullet.getQuaternionFromEuler(rot.get_rotation_as_list())
        object_id = pybullet.loadURDF(urdf_filepath, pos_as_list, rot_as_quat, useFixedBase=False)

        self._platforms.append(object_id)

        return object_id

    @staticmethod
    def add_sphere(position, color):
        position[2] += 0.1
        radius = 0.1
        mass = 1

        sphere_collision_shape = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=radius)
        sphere_visual_shape = pybullet.createVisualShape(pybullet.GEOM_SPHERE, radius=radius, rgbaColor=color + [1])

        pybullet.createMultiBody(baseMass=mass, baseCollisionShapeIndex=sphere_collision_shape,
                                 baseVisualShapeIndex=sphere_visual_shape, basePosition=position)

    @staticmethod
    def add_arrow(position, direction, color):
        pybullet.addUserDebugLine(np.array(position), direction, color, lineWidth=2.0)

    @staticmethod
    def step_simulation():
        tango = 0
        while pybullet.isConnected():
            pybullet.stepSimulation()
            time.sleep(1. / 240.)
            tango += 1

    @staticmethod
    def end_simulation():
        pybullet.disconnect()

    @staticmethod
    def add_force_to_object(object_id, force: NavForce):
        force_magnitude = force.get_force_magnitude()
        force_pos = force.get_force_pose().get_position().get_position_as_np_array()
        force_rot = force.get_force_pose().get_rotation().get_rotation_as_np_array() * 2

        pybullet.applyExternalForce(object_id, -1, force_rot, force_pos, pybullet.WORLD_FRAME)
