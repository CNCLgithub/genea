import crocoddyl
import numpy as np
import pinocchio
import time

from mlr.share.projects.navigation.model.jump_task import JumpTask
from mlr.share.projects.navigation.model.stimuli import Stimuli, StimuliPairs
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavAgent, NavTask
from mlr.share.projects.navigation.utils.pybullet_utils import PyBulletUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.crocoddyl_utils import CrocoddylUtils


class NavModel:
    def __init__(self, nav_agent_name, nav_scene_dirpath):
        self._agent = NavAgent(nav_agent_name)

        self._agent_init_pose = self._agent.get_x0()
        self._agent_current_pose = self._agent.get_x0()

        self._scene = Stimuli(nav_scene_dirpath)
        self._scene.load_stimuli_from_library()

        self._nav_task_list = []
        self._nav_task_solver = []

    def _add_solver(self, nav_task: NavTask):
        solver = crocoddyl.SolverFDDP(nav_task.get_task_solver(self._agent, self._agent_current_pose))
        solver.th_stop = ConfigUtils.NAV_MODEL_THRESHOLD_STOP

        if ConfigUtils.NAV_MODEL_SHOW_ROBOT_SIMULATION:
            solver.setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackLogger()])  # noqa
        else:
            solver.setCallbacks([crocoddyl.CallbackVerbose()])  # noqa

        _xs = [self._agent_current_pose] * (solver.problem.T + 1)
        _us = solver.problem.quasiStatic([self._agent_current_pose] * solver.problem.T)
        solver.solve(_xs, _us, 100, False)

        self._agent_current_pose = solver.xs[-1]
        self._nav_task_solver.append(solver)

    def add_jump_task(self):
        platform_names_list = self._scene.get_platform_names_list()
        platform_pairs = ComputeUtils.get_unique_combinations(platform_names_list, 2)

        jump_length = 0.0
        for idx1, idx2 in platform_pairs:
            jump_length += self._scene.get_gap_between_platforms(platform_names_list[idx1], platform_names_list[idx2])
        jump_length /= len(platform_pairs)
        jump_x, jump_y = ComputeUtils.sample_trunc_normal_circular(0.0, .6, 0.75)
        print(jump_x, jump_y, jump_length)
        jump_vector = np.array([jump_length + jump_x, jump_y, 0.0])

        jump_task = JumpTask()
        jump_task.set_jump_height(ConfigUtils.NAV_MODEL_JUMP_HEIGHT)
        jump_task.set_jump_vector(jump_vector)

        self._nav_task_list.append(jump_task)
        self._add_solver(jump_task)

    def run_dynamics(self):
        stimuli_dirpath = self._scene.get_stimuli_dirpath()
        _, ground_urdf_rel_filepath = self._scene.get_ground_urdf_filepath()

        for task_solver in self._nav_task_solver:
            nav_forces = CrocoddylUtils.get_forces_list(task_solver)

            pybullet_utils = PyBulletUtils(True)
            pybullet_utils.add_load_urdf(stimuli_dirpath, ground_urdf_rel_filepath, is_fixed=True)
            for platform_name in self._scene.get_platform_names_list():
                _, urdf_rel_filepath = self._scene.get_platform_urdf_filepath(platform_name)
                pybullet_utils.add_load_urdf(stimuli_dirpath, urdf_rel_filepath)

            platform_ids = pybullet_utils.get_platform_ids_list()
            for p_id, forces_list in zip(platform_ids, ComputeUtils.chunkify_list(nav_forces, len(platform_ids))):
                for forces in forces_list:
                    if len(forces) == 0:
                        continue
                    pybullet_utils.add_force_to_object(p_id, forces[0])  # force from the left foot
                    pybullet_utils.add_force_to_object(p_id, forces[1])  # force from the right foot

            pybullet_utils.step_simulation()
            pybullet_utils.end_simulation()

        if ConfigUtils.NAV_MODEL_SHOW_FORCE_SIMULATION:
            display = crocoddyl.MeshcatDisplay(self._agent.get_agent())
            display.rate = -1
            display.freq = 1
            while True:
                for task_solver in enumerate(self._nav_task_solver):
                    display.displayFromSolver(task_solver)
                time.sleep(1.0)

    def compute_stability(self):
        new_position, new_orientation = p.getBasePositionAndOrientation(boxId)


def run_pair_jumps():
    stimuli_pairs_dirpath_list = FileUtils.get_dir_list_in_directory(PathUtils.get_stimuli_pairs_dirpath())

    for stimuli_pair_dirpath in stimuli_pairs_dirpath_list:
        if "wide_top_wide_top" not in stimuli_pair_dirpath:
            continue
        nav_model = NavModel(NavAgent.TALOS_LEGS, stimuli_pair_dirpath)
        nav_model.add_jump_task()
        nav_model.run_dynamics()


if __name__ == '__main__':
    run_pair_jumps()
