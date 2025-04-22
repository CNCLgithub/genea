import crocoddyl
import hppfcl
import numpy as np
import pinocchio


class Platform:
    def __init__(self, name, position, rotation, shape, convex_hull):
        self.name = name
        self.position = position
        self.rotation = rotation
        self.shape = shape
        self.convex_hull = convex_hull

    def get_platform_position(self):
        return self.position

    def get_platform_shape(self):
        return self.shape

    def get_platform_convex_hull(self):
        return self.convex_hull

    def get_as_pinocchio_geom(self, robot_model):
        platform_pose = pinocchio.SE3.Identity()  # noqa
        platform_pose.translation = self.position
        platform_pose.rotation = self.rotation
        return pinocchio.GeometryObject(self.name,
                                        robot_model.getFrameId("universe"),
                                        robot_model.frames[robot_model.getFrameId("universe")].parent,
                                        platform_pose,
                                        hppfcl.Box(self.shape[0], self.shape[1], self.shape[2]))


class PlatformSet:
    def __init__(self):
        self.platforms_list = []

    def add_platform(self, position, rotation, shape, convex_hull):
        platform_name = "platform" + str(len(self.platforms_list))
        platform = Platform(platform_name, position, rotation, shape, convex_hull)
        self.platforms_list.append(platform)

    def get_total_platforms(self):
        return len(self.platforms_list)

    def get_platforms_list(self):
        return self.platforms_list

    def get_platform_by_name(self, name):
        for platform in self.platforms_list:
            if platform.name == name:
                return platform
        raise ValueError(f"ERROR[PlatformSet]: platform with name {name} was not found.")


class JumpProblem:
    JUMP_VEC = np.array([2.5, 0.0, 0])
    JUMP_HEIGHT = 2.5

    TIME_STEP = 0.02
    GROUND_KNOTS = 20
    FLYING_KNOTS = 20


class GeneaJumpProblem:
    def __init__(self, robot_model, right_foot_link, left_foot_link, platform_set):
        self.robot_model = robot_model
        self.robot_data = robot_model.createData()

        self.state = crocoddyl.StateMultibody(self.robot_model)
        self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)

        self.platform_set = platform_set

        self.right_foot_link = right_foot_link
        self.left_foot_link = left_foot_link
        self.foot_links_list = [self.right_foot_link, self.left_foot_link]

        self.right_frame_id = self.robot_model.getFrameId(right_foot_link)
        self.left_frame_id = self.robot_model.getFrameId(left_foot_link)
        self.foot_frame_ids_list = [self.right_frame_id, self.left_frame_id]

        self.right_foot_pos = self.robot_data.oMf[self.right_frame_id].translation
        self.left_foot_pos = self.robot_data.oMf[self.left_frame_id].translation
        self.foot_pos_list = [self.right_foot_pos, self.left_foot_pos]

        q0 = self.robot_model.referenceConfigurations["half_sitting"]
        self.robot_model.defaultState = np.concatenate([q0, np.zeros(self.robot_model.nv)])

        self.friction_coefficient = 0.7
        self.surface_normal = np.eye(3)

    def create_jump_problem(self, x0):
        pose0 = x0[: self.robot_model.nq]
        pinocchio.forwardKinematics(self.robot_model, self.robot_data, pose0)
        pinocchio.updateFramePlacements(self.robot_model, self.robot_data)

        height_diff = JumpProblem.JUMP_VEC[2] - self.right_foot_pos[2]

        self.right_foot_pos[2] = 0.0
        self.left_foot_pos[2] = 0.0

        com_ref = (self.right_foot_pos + self.left_foot_pos) / 2
        com_ref[2] = pinocchio.centerOfMass(self.robot_model, self.robot_data, pose0)[2]

        com_jump = np.array([JumpProblem.JUMP_VEC[0] / 2.0,
                             JumpProblem.JUMP_VEC[1] / 2.0,
                             JumpProblem.JUMP_VEC[2] / 2.0 + JumpProblem.JUMP_HEIGHT])

        loco_problem_list = []

        take_off_phase = []
        for _ in range(JumpProblem.GROUND_KNOTS):
            take_off_phase.append(self.get_swing_foot_model())

        fly_up_phase = []
        for k in range(JumpProblem.FLYING_KNOTS):
            com_jump_segment = com_jump * (k + 1) / JumpProblem.FLYING_KNOTS + com_ref
            fly_up_phase.append(self.get_swing_foot_model(add_contact=False, com_pos=com_jump_segment))

        fly_down_phase = []
        for k in range(JumpProblem.FLYING_KNOTS):
            fly_down_phase.append(self.get_swing_foot_model(add_contact=False))

        touch_down_phase = [self.get_impulse_model()]

        land_phase = []
        land_vec = JumpProblem.JUMP_VEC.copy()
        land_vec[2] = height_diff
        for _ in range(int(JumpProblem.GROUND_KNOTS / 2)):
            land_phase.append(self.get_swing_foot_model(com_pos=com_ref + land_vec))

        loco_problem_list += take_off_phase
        loco_problem_list += fly_up_phase
        loco_problem_list += fly_down_phase
        loco_problem_list += touch_down_phase
        loco_problem_list += land_phase

        return crocoddyl.ShootingProblem(x0, loco_problem_list[:-1], loco_problem_list[-1])

    def get_swing_foot_model(self, add_contact=True, com_pos=None):
        costs_list = crocoddyl.CostModelSum(self.state, self.actuation.nu)

        if isinstance(com_pos, np.ndarray):
            com_residual = crocoddyl.ResidualModelCoMPosition(self.state, com_pos, self.actuation.nu)
            com_cost = crocoddyl.CostModelResidual(self.state, com_residual)
            costs_list.addCost("com_track", com_cost, 1e6)

        for frame_id in self.foot_frame_ids_list:
            if not add_contact:
                continue
            w_cone = crocoddyl.WrenchCone(self.surface_normal, self.friction_coefficient, np.array([0.1, 0.05]))
            w_cone_res = crocoddyl.ResidualModelContactWrenchCone(self.state, frame_id, w_cone, self.actuation.nu)
            w_cone_act = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(w_cone.lb, w_cone.ub))
            w_cone_cost = crocoddyl.CostModelResidual(self.state, w_cone_act, w_cone_res)
            costs_list.addCost(self.robot_model.frames[frame_id].name + "_wrench_cone", w_cone_cost, 1e1)

        state_weights = np.array([0] * 3 + [500.0] * 3 + [0.01] * (self.state.nv - 6) + [10] * self.state.nv)
        state_residual = crocoddyl.ResidualModelState(self.state, self.robot_model.defaultState, self.actuation.nu)
        state_activation = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        state_cost = crocoddyl.CostModelResidual(self.state, state_activation, state_residual)
        costs_list.addCost("state_cost", state_cost, 1e1)

        ctrl_residual = crocoddyl.ResidualModelControl(self.state, self.actuation.nu)
        ctrl_cost = crocoddyl.CostModelResidual(self.state, ctrl_residual)
        costs_list.addCost("ctrl_cost", ctrl_cost, 1e-1)

        contacts_list = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
        for frame_id in self.foot_frame_ids_list:
            if not add_contact:
                continue
            contact_model = crocoddyl.ContactModel6D(self.state,
                                                     frame_id,
                                                     pinocchio.SE3.Identity(),  # noqa
                                                     pinocchio.LOCAL_WORLD_ALIGNED,
                                                     self.actuation.nu,
                                                     np.array([0.0, 30.0]))
            contacts_list.addContact(self.robot_model.frames[frame_id].name + "_contact", contact_model)

        if add_contact:
            self._get_collision_model(costs_list)

        diff_action_model = crocoddyl.DifferentialActionModelContactFwdDynamics(self.state, self.actuation,
                                                                                contacts_list, costs_list, 0.0, True)

        control = crocoddyl.ControlParametrizationModelPolyZero(self.actuation.nu)

        return crocoddyl.IntegratedActionModelEuler(diff_action_model, control, JumpProblem.TIME_STEP)

    def get_impulse_model(self):
        costs_list = crocoddyl.CostModelSum(self.state, 0)

        for foot_id, foot_pos0 in zip(self.foot_frame_ids_list, self.foot_pos_list):
            pos = pinocchio.SE3(np.eye(3), foot_pos0 + JumpProblem.JUMP_VEC)
            fp_residual = crocoddyl.ResidualModelFramePlacement(self.state, foot_id, pos, 0)
            fp_cost = crocoddyl.CostModelResidual(self.state, fp_residual)
            costs_list.addCost(self.robot_model.frames[foot_id].name + "_foot_track", fp_cost, 1e8)

        state_weights = np.array([1.0] * 6 + [0.1] * (self.robot_model.nv - 6) + [10] * self.robot_model.nv)
        state_activation = crocoddyl.ActivationModelWeightedQuad(state_weights**2)
        state_residual = crocoddyl.ResidualModelState(self.state, self.robot_model.defaultState, 0)
        state_cost = crocoddyl.CostModelResidual(self.state, state_activation, state_residual)
        costs_list.addCost("state_cost", state_cost, 1e1)

        impulses_list = crocoddyl.ImpulseModelMultiple(self.state)
        for supp_foot_id in self.foot_frame_ids_list:
            contact_model = crocoddyl.ImpulseModel6D(self.state, supp_foot_id, pinocchio.LOCAL_WORLD_ALIGNED)
            impulses_list.addImpulse(self.robot_model.frames[supp_foot_id].name + "_impulse", contact_model)

        impulse_action_model = crocoddyl.ActionModelImpulseFwdDynamics(self.state, impulses_list, costs_list)
        impulse_action_model.JMinvJt_damping = 1e-12
        impulse_action_model.r_coeff = 0.0
        return impulse_action_model

    def _get_collision_model(self, costs_list):
        geom_model = pinocchio.GeometryModel()  # noqa

        for foot_link_name, foot_frame_id in zip(self.foot_links_list, self.foot_frame_ids_list):
            geom_model.addGeometryObject(pinocchio.GeometryObject(foot_link_name,
                                                                  foot_frame_id,
                                                                  self.robot_model.frames[foot_frame_id].parentJoint,
                                                                  pinocchio.SE3.Identity(),  # noqa
                                                                  hppfcl.Capsule(0, 0.5)))

        for foot_link in self.foot_links_list:
            foot_geom_id = geom_model.getGeometryId(foot_link)
            for platform in self.platform_set.get_platforms_list():
                platform_geom_id = geom_model.addGeometryObject(platform.get_as_pinocchio_geom(self.robot_model))
                geom_model.addCollisionPair(pinocchio.CollisionPair(foot_geom_id, platform_geom_id))

        for pair_id, collision_pair in enumerate(geom_model.collisionPairs):
            foot_link_id = collision_pair.first
            platform = self.platform_set.get_platform_by_name(geom_model.geometryObjects[collision_pair.second].name)

            nu = self.actuation.nu

            collision_act = crocoddyl.ActivationModel2NormBarrier(3, platform.get_platform_convex_hull())
            collision_res = crocoddyl.ResidualModelPairCollision(self.state, nu, geom_model, pair_id, foot_link_id)
            collision_cost = crocoddyl.CostModelResidual(self.state, collision_act, collision_res)
            costs_list.addCost("collision_" + str(pair_id), collision_cost, 1e6)
