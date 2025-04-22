import crocoddyl
import example_robot_data
import numpy as np
import pinocchio
import signal
import time

from mlr.share.projects.navigation.model.jump_problem import GeneaJumpProblem, PlatformSet
from mlr.share.projects.navigation.utils.navigation_utils import NavForce, NavPosition, NavRotation
from mlr.share.projects.navigation.utils.pybullet_utils import PyBulletUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.crocoddyl_utils import CrocoddylUtils

show_display = True
show_plot = False
robot_tasks = ["jump", "jump", "jump"]


signal.signal(signal.SIGINT, signal.SIG_DFL)


talos_legs = example_robot_data.load("talos_legs")
q0 = talos_legs.model.referenceConfigurations["half_sitting"].copy()

v0 = pinocchio.utils.zero(talos_legs.model.nv)
x0 = np.concatenate([q0, v0])

platform_rotation = pinocchio.SE3.Identity().rotation
platform_shape = np.array([0.5, 1.0, 0.5])
platform_set = PlatformSet()
platform_set.add_platform(np.array([0.0, 0, -0.25]), platform_rotation, platform_shape, 0.15)
platform_set.add_platform(np.array([2.5, 0, -0.25]), platform_rotation, platform_shape, 0.15)

for platform in platform_set.get_platforms_list():
    talos_legs.visual_model.addGeometryObject(platform.get_as_pinocchio_geom(talos_legs.model))

rightFoot = "right_sole_link"
leftFoot = "left_sole_link"
genea_jump_problem = GeneaJumpProblem(talos_legs.model, rightFoot, leftFoot, platform_set)

solver = []
for i, phase in enumerate(robot_tasks):
    if phase == "jump":
        jump_problem = genea_jump_problem.create_jump_problem(x0)
        solver.append(crocoddyl.SolverFDDP(jump_problem))
    else:
        continue

    solver[i].th_stop = 1e-7

    if show_plot:
        solver[i].setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackLogger()])  # noqa
    else:
        solver[i].setCallbacks([crocoddyl.CallbackVerbose()])  # noqa

    xs = [x0] * (solver[i].problem.T + 1)
    us = solver[i].problem.quasiStatic([x0] * solver[i].problem.T)
    solver[i].solve(xs, us, 100, False)

    x0 = solver[i].xs[-1]

nav_forces = CrocoddylUtils.get_forces_list(solver[0])

mesh_dirpath = PathUtils.get_platforms_dirpath()
mesh_filepath = "urdf/basic.urdf"
pybullet_utils = PyBulletUtils(True)
pybullet_utils.add_ground_plane()
pid = pybullet_utils.add_object_from_urdf(mesh_dirpath, mesh_filepath, NavPosition(0, 0, 0), NavRotation(0, 0, 0))

for force_id, force in enumerate(nav_forces):
    if len(force) == 0:
        break
    pybullet_utils.add_force_to_object(pid, force[0])
    pybullet_utils.add_force_to_object(pid, force[1])

pybullet_utils.step_simulation()
pybullet_utils.end_simulation()

# if show_display:
#     display = crocoddyl.MeshcatDisplay(talos_legs)
#     display.rate = -1
#     display.freq = 1
#     while True:
#         for i, phase in enumerate(robot_tasks):
#             print([phase])
#             display.displayFromSolver(solver[i])
#         time.sleep(1.0)
