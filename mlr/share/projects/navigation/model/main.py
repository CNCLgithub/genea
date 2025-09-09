import crocoddyl
import numpy as np
import time

from mlr.share.projects.navigation.model.jump_task import JumpTask
from mlr.share.projects.navigation.model.stimuli import StimuliPairs
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.navigation_utils import NavAgent, NavTask
from mlr.share.projects.navigation.utils.pybullet_utils import PyBulletUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.crocoddyl_utils import CrocoddylUtils
from mlr.share.projects.navigation.utils.stimuli_utils import StimulusItem


class NavModel:
    def __init__(self, nav_agent_name, nav_scene_dirpath, out_filepath):
        self._agent = NavAgent(nav_agent_name)

        self._agent_init_pose = self._agent.get_x0()
        self._agent_current_pose = self._agent.get_x0()

        self._scene = StimulusItem(nav_scene_dirpath, StimuliPairs())
        self._scene.load_stimuli_from_library()

        self._nav_task_list = []

        self._out_filepath = out_filepath

    def _setup_task_solver(self, nav_task: NavTask):
        solver = crocoddyl.SolverFDDP(nav_task.get_task_problem(self._agent, self._agent_current_pose))
        solver.th_stop = ConfigUtils.NAV_MODEL_THRESHOLD_STOP

        if ConfigUtils.NAV_MODEL_VIEW_DYNAMICS:
            solver.setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackLogger()])  # noqa
        else:
            solver.setCallbacks([crocoddyl.CallbackVerbose()])  # noqa

        _xs = [self._agent_current_pose] * (solver.problem.T + 1)
        _us = solver.problem.quasiStatic([self._agent_current_pose] * solver.problem.T)
        solver.solve(_xs, _us, 100, False)

        self._agent_current_pose = solver.xs[-1]

        nav_task.set_task_solver(solver)

    def _view_kinematics(self):
        display = crocoddyl.MeshcatDisplay(self._agent.get_agent())
        display.rate = -1
        display.freq = 1
        while True:
            for nav_task in self._nav_task_list:
                display.displayFromSolver(nav_task.get_task_solver())
            time.sleep(1.0)

    @staticmethod
    def has_stimuli_moved(start_pose, final_pose):
        pos_diff, rot_diff = PyBulletUtils.get_pose_diff(start_pose, final_pose)

        if pos_diff < ConfigUtils.PYBULLET_POS_THRESHOLD or rot_diff < ConfigUtils.PYBULLET_ROT_THRESHOLD:
            Msg.print_info(f"INFO [NavModel]: platform stable -- pos={pos_diff}, rot={rot_diff}")
            return False, pos_diff, rot_diff

        Msg.print_info(f"INFO [NavModel]: platform has moved (unstable) -- pos={pos_diff}, rot={rot_diff}")
        return True, pos_diff, rot_diff

    def add_jump_task(self):
        platform_keys_list = self._scene.get_platform_keys_list()
        platform_key1, platform_key2 = platform_keys_list[0], platform_keys_list[1]

        jump_length = 0.0
        jump_length += self._scene.get_gap_between_platforms(platform_key1, platform_key2)

        platform_x, platform_y = self._scene.get_platform_surface_measures(platform_key2)
        platform_x /= 2.0
        platform_y /= 2.0

        jump_x = ComputeUtils.sample_trunc_normal(0.0, -platform_x, platform_x, 1.0)
        jump_y = ComputeUtils.sample_trunc_normal(0.0, -platform_y, platform_y, 1.0)
        jump_vector = np.array([jump_length + jump_x, jump_y, 0.0])

        print(jump_vector)

        jump_task = JumpTask()
        jump_task.set_jump_height(ConfigUtils.NAV_MODEL_JUMP_HEIGHT)
        jump_task.set_jump_vector(jump_vector)

        self._setup_task_solver(jump_task)
        self._nav_task_list.append(jump_task)

    def run_dynamics(self):
        if ConfigUtils.NAV_MODEL_VIEW_KINEMATICS:
            self._view_kinematics()

        ground_rel_filepath = self._scene.get_urdf_rel_filepath(self._scene.get_platform_ground_key())

        for nav_task in self._nav_task_list:
            nav_forces = CrocoddylUtils.get_forces_list(nav_task.get_task_solver())

            pybullet_utils = PyBulletUtils(ConfigUtils.NAV_MODEL_VIEW_DYNAMICS)
            pybullet_utils.add_load_urdf(self._scene.get_stimulus_item_dirpath(), ground_rel_filepath, is_fixed=True)
            for platform_key in self._scene.get_platform_keys_list():
                platform_rel_filepath = self._scene.get_urdf_rel_filepath(platform_key)
                pybullet_utils.add_load_urdf(self._scene.get_stimulus_item_dirpath(), platform_rel_filepath)

            platform_ids = pybullet_utils.get_platform_ids_list()

            # store poses before dynamics
            for p_key, p_id in zip(self._scene.get_platform_keys_list(), platform_ids):
                self._scene.get_platform_by_key(p_key).add_platform_pose(pybullet_utils.get_pose(p_id))

            for p_id, forces_list in zip(platform_ids, ComputeUtils.chunkify_list(nav_forces, len(platform_ids))):
                for forces in forces_list:
                    if len(forces) == 0:
                        continue
                    pybullet_utils.add_force_to_object(p_id, forces[0])  # force from the left foot
                    pybullet_utils.add_force_to_object(p_id, forces[1])  # force from the right foot

            pybullet_utils.run_simulation(ConfigUtils.PYBULLET_SIMULATION_TIME)

            # store poses before dynamics
            for p_key, p_id in zip(self._scene.get_platform_keys_list(), platform_ids):
                self._scene.get_platform_by_key(p_key).add_platform_pose(pybullet_utils.get_pose(p_id))

            pybullet_utils.close()

    def save_result_to_outfile(self, run_num):
        for nav_task in self._nav_task_list:
            for platform_key in self._scene.get_platform_keys_list():
                platform = self._scene.get_platform_by_key(platform_key)

                start_pose = platform.get_platform_pose(0)
                final_pose = platform.get_platform_pose(-1)
                has_moved, pos_diff, rot_diff = self.has_stimuli_moved(start_pose, final_pose)

                out_data = [self._scene.get_stimulus_item_name(),
                            nav_task.get_task_type(),
                            platform_key,
                            run_num,
                            nav_task.get_task_cost(),
                            pos_diff,
                            rot_diff,
                            has_moved]

                FileUtils.write_row_to_file(self._out_filepath, out_data)


def run_pair_jumps(total_runs=10):
    nav_agent_name = NavAgent.TALOS_LEGS

    out_dirpath = PathUtils.join(PathUtils.get_out_dirpath(), nav_agent_name)
    FileUtils.create_dir(out_dirpath, do_force_create=False)
    out_filepath = PathUtils.join(out_dirpath, "out_data.csv")

    stimuli_pairs_dirpath_list = FileUtils.get_dir_list_in_directory(StimuliPairs().get_stimuli_set_dirpath())

    for stimuli_pair_dirpath in stimuli_pairs_dirpath_list:
        if "cube_std_long_std" not in FileUtils.get_basename(stimuli_pair_dirpath):
            continue
        for run_num in range(1, total_runs):
            nav_model = NavModel(NavAgent.TALOS_LEGS, stimuli_pair_dirpath, out_filepath)
            nav_model.add_jump_task()
            nav_model.run_dynamics()
            nav_model.save_result_to_outfile(run_num)


if __name__ == '__main__':
    run_pair_jumps()
