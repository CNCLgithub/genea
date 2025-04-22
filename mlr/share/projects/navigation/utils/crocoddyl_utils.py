import numpy as np
import pinocchio

from crocoddyl import DifferentialActionDataContactFwdDynamics, DifferentialActionDataContactInvDynamics, \
    ActionDataImpulseFwdDynamics, StdVec_DiffActionData, ActionModelImpulseFwdDynamics

from mlr.share.projects.navigation.utils.config_utils import NavConfigUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavForce, NavPose, NavPosition, NavRotation


class CrocoddylUtils:
    @staticmethod
    def _has_contacts(data):
        if hasattr(data, "differential"):
            condition1 = isinstance(data.differential, DifferentialActionDataContactFwdDynamics)
            condition2 = isinstance(data.differential, DifferentialActionDataContactInvDynamics)
            if condition1 or condition2:
                return True
        elif isinstance(data, ActionDataImpulseFwdDynamics):
            return True
        return False

    @staticmethod
    def _get_contact_model_data(model, data):
        if hasattr(data, "differential"):
            condition1 = isinstance(data.differential, DifferentialActionDataContactFwdDynamics)
            condition2 = isinstance(data.differential, DifferentialActionDataContactInvDynamics)
            condition3 = isinstance(data.differential, StdVec_DiffActionData)
            if condition1 or condition2:
                return model.differential.contacts.contacts, data.differential.multibody.contacts.contacts
            elif condition3 and (condition1 or condition2):
                return model.differential[0].contacts.contacts, data.differential[0].multibody.contacts.contacts
        elif isinstance(data, ActionDataImpulseFwdDynamics):
            return model.impulses.impulses, data.multibody.impulses.impulses
        return False

    @staticmethod
    def _get_cost_model(model):
        if hasattr(model, "differential"):
            return model.differential.costs.costs
        elif isinstance(model, ActionModelImpulseFwdDynamics):
            return model.costs.costs
        return False

    @staticmethod
    def _get_forces_list(state, contact_model, contact_data):
        forces_list = []
        for key, contact in contact_data.todict().items():
            if contact_model[key].active:
                joint = state.pinocchio.frames[contact.frame].parentJoint
                body_pose = pinocchio.SE3(contact.pinocchio.oMi[joint].rotation.T, contact.jMf.translation)
                force_wrench = body_pose.actInv(contact.fext)

                force_pos = (contact.pinocchio.oMi[joint] * contact.jMf).translation
                nav_pos = NavPosition(force_pos[0], force_pos[1], force_pos[2])

                force_rot = -force_wrench.linear
                nav_rot = NavRotation(force_rot[0], force_rot[1], force_rot[2])

                force_pose = NavPose(nav_pos, nav_rot)
                force_magnitude = np.linalg.norm(force_wrench.linear) / NavConfigUtils.FORCE_SCALING_FACTOR
                forces_list.append(NavForce(force_pose, force_magnitude))

        return forces_list

    @staticmethod
    def get_forces_list(crocoddyl_solver):
        forces_list = []
        model_list = [*crocoddyl_solver.problem.runningModels.tolist(), crocoddyl_solver.problem.terminalModel]
        data_list = [*crocoddyl_solver.problem.runningDatas.tolist(), crocoddyl_solver.problem.terminalData]
        for model, data in zip(model_list, data_list):
            if CrocoddylUtils._has_contacts(data):
                contact_model, contact_data = CrocoddylUtils._get_contact_model_data(model, data)
                forces_list.append(CrocoddylUtils._get_forces_list(model.state, contact_model, contact_data))
        return forces_list

    @staticmethod
    def get_costs_list(crocoddyl_solver):
        costs_list = []
        model_list = [*crocoddyl_solver.problem.runningModels.tolist(), crocoddyl_solver.problem.terminalModel]
        data_list = [*crocoddyl_solver.problem.runningDatas.tolist(), crocoddyl_solver.problem.terminalData]
        for model, data in zip(model_list, data_list):
            if CrocoddylUtils._has_contacts(data):
                costs_list.append(CrocoddylUtils._get_cost_model(model))
        return costs_list
