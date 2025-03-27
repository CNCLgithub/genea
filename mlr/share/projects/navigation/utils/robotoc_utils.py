import robotoc

from pinocchio import RobotWrapper, JointModelFreeFlyer

from mlr.share.projects.navigation.utils.config_utils import NavConfigUtils


class Robotoc:
    @staticmethod
    def get_agent_from_urdf(urdf_filepath, urdf_root_dirpath):
        model_info = robotoc.RobotModelInfo()
        model_info.urdf_path = urdf_filepath
        model_info.base_joint_type = robotoc.BaseJointType.FloatingBase

        model_info.surface_contacts = [robotoc.ContactModelInfo('l_sole', NavConfigUtils.ROBOTOC_TIME_STEP),
                                       robotoc.ContactModelInfo('r_sole', NavConfigUtils.ROBOTOC_TIME_STEP)]

        RobotWrapper.BuildFromURDF(model_info.urdf_path, urdf_root_dirpath, JointModelFreeFlyer())

        return robotoc.Robot(model_info)
