import crocoddyl
import numpy as np
import time

from enum import Enum

from mlr.share.projects.navigation.model.stimuli import StimuliPairs
from mlr.share.projects.navigation.model.tasks.jump import JumpTask
from mlr.share.projects.navigation.model.tasks.walk import WalkTask
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import ConfigUtils
from mlr.share.projects.navigation.utils.crocoddyl_utils import CrocoddylUtils
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.navigation_utils import NavAgent, NavTask
from mlr.share.projects.navigation.utils.path_utils import PathUtils
from mlr.share.projects.navigation.utils.pybullet_utils import PyBulletUtils
from mlr.share.projects.navigation.utils.stimuli_utils import StimulusItem


class NavOutLabel(Enum):
    NAV_STIM_ITEM_NAME = "nav_stim_item_name"
    NAV_AGENT_NAME = "nav_agent"
    NAV_TASK = "nav_task"
    RUN_NUM = "run_num"
    NAV_TASK_COST = "nav_task_cost"
    NAV_TASK_INSTABILITY = "nav_task_instability"

    PLATFORM_NAME = "platform_name"
    PLATFORM_POS_DIFF = "platform_pos_diff"
    PLATFORM_ROT_DIFF = "platform_rot_diff"
    PLATFORM_HAS_MOVED = "platform_has_moved"
    PLATFORM_MASS = "platform_mass"


class NavOutData:
    def __init__(self, stim_set_name=None):
        self._stim_set_name = stim_set_name

        self._nav_data_dict = {}
        self._platform_data_list = []

    def get_nav_out_filepath(self):
        FileUtils.create_dir(PathUtils.get_out_nav_data_dirpath(), do_force_create=False)
        return PathUtils.join(PathUtils.get_out_nav_data_dirpath(), self._stim_set_name + ".csv")

    def get_nav_data(self, key: NavOutLabel):
        return self._nav_data_dict.get(key, None)

    def add_nav_data(self, key: NavOutLabel, value):
        self._nav_data_dict[key] = value

    def add_platform_data(self, platform_name, platform_pos_diff, platform_rot_diff, platform_has_moved, platform_mass):
        self._platform_data_list.append({NavOutLabel.PLATFORM_NAME: platform_name,
                                         NavOutLabel.PLATFORM_POS_DIFF: platform_pos_diff,
                                         NavOutLabel.PLATFORM_ROT_DIFF: platform_rot_diff,
                                         NavOutLabel.PLATFORM_HAS_MOVED: platform_has_moved,
                                         NavOutLabel.PLATFORM_MASS: platform_mass})

    def save_nav_out_data_to_file(self):
        nav_out_data_as_list = [self._nav_data_dict[key] for key in [NavOutLabel.NAV_STIM_ITEM_NAME,
                                                                     NavOutLabel.NAV_AGENT_NAME,
                                                                     NavOutLabel.NAV_TASK,
                                                                     NavOutLabel.RUN_NUM]]

        for platform_data in self._platform_data_list:
            nav_out_data_as_list.append(platform_data[NavOutLabel.PLATFORM_NAME])
            nav_out_data_as_list.append(platform_data[NavOutLabel.PLATFORM_POS_DIFF])
            nav_out_data_as_list.append(platform_data[NavOutLabel.PLATFORM_ROT_DIFF])
            nav_out_data_as_list.append(platform_data[NavOutLabel.PLATFORM_HAS_MOVED])
            nav_out_data_as_list.append(platform_data[NavOutLabel.PLATFORM_MASS])

        nav_out_data_as_list.append(self._nav_data_dict[NavOutLabel.NAV_TASK_COST])
        nav_out_data_as_list.append(self._nav_data_dict[NavOutLabel.NAV_TASK_INSTABILITY])

        FileUtils.write_row_to_file(self.get_nav_out_filepath(), nav_out_data_as_list)

    def parse_row_data(self, row_data):
        self._nav_data_dict[NavOutLabel.NAV_STIM_ITEM_NAME] = row_data[0]
        self._nav_data_dict[NavOutLabel.NAV_AGENT_NAME] = row_data[1]
        self._nav_data_dict[NavOutLabel.NAV_TASK] = row_data[2]
        self._nav_data_dict[NavOutLabel.RUN_NUM] = int(row_data[3])

        for index in range(4, len(row_data) - 2, 4):
            platform_data = row_data[index: index + 4]

            p_name = platform_data[0]
            p_pos_diff = float(platform_data[1])
            p_rot_diff = float(platform_data[2])
            p_has_moved = platform_data[3] == 'True'
            p_mass = platform_data[4]
            self.add_platform_data(p_name, p_pos_diff, p_rot_diff, p_has_moved, p_mass)

        self._nav_data_dict[NavOutLabel.NAV_TASK_COST] = row_data[-2]
        self._nav_data_dict[NavOutLabel.NAV_TASK_INSTABILITY] = row_data[-1]


class NavModel:
    def __init__(self, nav_agent_name, nav_scene: StimulusItem):
        self._agent = NavAgent(nav_agent_name)

        self._agent_init_pose = self._agent.get_x0()
        self._agent_current_pose = self._agent.get_x0()

        self._scene = nav_scene
        self._scene.load_stimuli_from_library()

        self._nav_task_list = []

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
        pos_diff, rot_diff = start_pose.get_pose_diff(final_pose)

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

        jump_task = JumpTask()
        jump_task.set_jump_height(ConfigUtils.NAV_MODEL_JUMP_HEIGHT)
        jump_task.set_jump_vector(jump_vector)

        self._setup_task_solver(jump_task)
        self._nav_task_list.append(jump_task)

    def add_walk_task(self):
        platform_keys_list = self._scene.get_platform_keys_list()
        platform_key1 = platform_keys_list[0]

        platform_x_len = self._scene.get_platform_surface_measures(platform_key1)[0]

        walk_distance = 0.0
        walk_distance += platform_x_len
        walk_distance -= platform_x_len / 4

        total_steps = max(1, int(walk_distance // ConfigUtils.NAV_MODEL_STEP_LENGTH))
        step_length = walk_distance / total_steps

        for step_num in range(total_steps):
            walk_task = WalkTask(step_num == 0)
            walk_task.set_step_height(ConfigUtils.NAV_MODEL_STEP_HEIGHT)
            walk_task.set_step_length(step_length)

            self._setup_task_solver(walk_task)
            self._nav_task_list.append(walk_task)

    def run_dynamics(self):
        if ConfigUtils.NAV_MODEL_VIEW_KINEMATICS:
            self._view_kinematics()

        jump_onset_time = 0.0
        jump_stint_time = ConfigUtils.PYBULLET_SIM_JUMP * ConfigUtils.PYBULLET_SIM_TIME_STEP

        walk_onset_time = 0.0
        walk_stint_time = ConfigUtils.PYBULLET_SIM_WALK * ConfigUtils.PYBULLET_SIM_TIME_STEP

        ground_rel_filepath = self._scene.get_urdf_rel_filepath(self._scene.get_platform_ground_key())

        pybullet_utils = PyBulletUtils(ConfigUtils.NAV_MODEL_VIEW_DYNAMICS)
        pybullet_utils.add_load_urdf(self._scene.get_stimulus_item_dirpath(), ground_rel_filepath, is_fixed=True)
        for platform_key in self._scene.get_platform_keys_list():
            platform_rel_filepath = self._scene.get_urdf_rel_filepath(platform_key)
            pybullet_utils.add_load_urdf(self._scene.get_stimulus_item_dirpath(), platform_rel_filepath)

        p_id_list = pybullet_utils.get_platform_ids_list()

        for nav_task in self._nav_task_list:
            nav_forces = CrocoddylUtils.get_forces_list(nav_task.get_task_solver())

            # store poses before dynamics
            for p_key, p_id in zip(self._scene.get_platform_keys_list(), p_id_list):
                self._scene.get_platform_by_key(p_key).add_platform_pose(pybullet_utils.get_platform_pose(p_id))

            for p_id, nf_list in zip(p_id_list, ComputeUtils.chunkify_list(nav_forces, len(p_id_list))):
                for nfs in nf_list:
                    if len(nfs) == 0:
                        continue
                    if nav_task.get_task_type() == NavTask.JUMP:
                        pybullet_utils.add_force_on_platform(p_id, nfs[0], jump_onset_time, jump_stint_time)
                        pybullet_utils.add_force_on_platform(p_id, nfs[1], jump_onset_time, jump_stint_time)
                    elif nav_task.get_task_type() == NavTask.WALK:
                        if len(nfs) == 2:
                            pybullet_utils.add_force_on_platform(p_id, nfs[0], walk_onset_time, walk_stint_time)
                            pybullet_utils.add_force_on_platform(p_id, nfs[1], walk_onset_time, walk_stint_time)
                            continue
                        pybullet_utils.add_force_on_platform(p_id, nfs[0], walk_onset_time, walk_stint_time)

            walk_onset_time += walk_stint_time

            pybullet_utils.run_simulation()

            # store poses after dynamics
            for p_key, p_id in zip(self._scene.get_platform_keys_list(), p_id_list):
                self._scene.get_platform_by_key(p_key).add_platform_pose(pybullet_utils.get_platform_pose(p_id))

        pybullet_utils.close()

    def save_result_to_outfile(self, run_num):
        for nav_task in self._nav_task_list:
            nav_out_data = NavOutData(StimuliPairs().get_stimuli_set_name())
            nav_out_data.add_nav_data(NavOutLabel.NAV_STIM_ITEM_NAME, self._scene.get_stimulus_item_name(True))
            nav_out_data.add_nav_data(NavOutLabel.NAV_AGENT_NAME, self._agent.get_name())
            nav_out_data.add_nav_data(NavOutLabel.NAV_TASK, nav_task.get_task_type())
            nav_out_data.add_nav_data(NavOutLabel.RUN_NUM, run_num)
            
            nav_task_cost = 0.0
            nav_task_instability = 0

            for platform_key in self._scene.get_platform_keys_list():
                platform = self._scene.get_platform_by_key(platform_key)

                start_pose = platform.get_platform_pose(0)
                final_pose = platform.get_platform_pose(-1)
                has_moved, pos_diff, rot_diff = self.has_stimuli_moved(start_pose, final_pose)
                mass = ConfigUtils.STIMULI_PLATFORM_MASS

                nav_out_data.add_platform_data(platform.get_platform_name(), pos_diff, rot_diff, has_moved, mass)
                nav_task_cost += nav_task.get_task_cost()
                if has_moved:
                    nav_task_instability += 1

            nav_task_cost /= len(self._scene.get_platform_keys_list())
                    
            nav_out_data.add_nav_data(NavOutLabel.NAV_TASK_COST, nav_task_cost)
            nav_out_data.add_nav_data(NavOutLabel.NAV_TASK_INSTABILITY, nav_task_instability)

            nav_out_data.save_nav_out_data_to_file()


# class NavPlanner:
#     def __init__(self, stim_item: StimulusItem):
#         self._stim_item = stim_item
#
#         self._stim_platform_pairs_dirpath_list = self._load_stim_platform_pairs_dirpath_list()
#
#     def _load_stim_platform_pairs_dirpath_list(self):
#         platform_pairs_dirpath_list = []
#
#         stim_platform_keys_list = self._stim_item.get_platform_keys_list()
#         stim_platform_keys_list = [key for key in stim_platform_keys_list if StimulusItem.is_platform_movable(key)]
#
#         for platform_key1, platform_key2 in zip(stim_platform_keys_list, stim_platform_keys_list[1:]):
#             platform1 = self._stim_item.get_platform_by_key(platform_key1)
#             platform2 = self._stim_item.get_platform_by_key(platform_key2)
#
#             platform_pairs_name = StimuliPairs.get_platform_pairs_name(platform1, platform2)
#             platform_pairs_dirpath = PathUtils.join(StimuliPairs().get_stimuli_set_dirpath(), platform_pairs_name)
#
#             platform_pairs_dirpath_list.append(platform_pairs_dirpath)
#
#         return platform_pairs_dirpath_list
#
#     def _run_stim_pair(self, stim_platform_pairs_dirpath):
#         nav_model = NavModel(NavAgent.TALOS_LEGS, stim_platform_pairs_dirpath)
#         nav_model.run_dynamics()
#
#     def run_planner(self):
#         for stim_pair_dirpath in self._stim_platform_pairs_dirpath_list:



    # @staticmethod
    # def _read_platform_pair_data():
    #     nav_csv_data = FileUtils.read_csv_file(NavOutData(StimuliPairs().get_stimuli_set_name()).get_nav_out_filepath())
    #
    #     nav_out_data = NavOutData()
    #     for row_data in nav_csv_data:
    #         nav_out_data.parse_row_data(row_data)
    #
    #     return nav_out_data
    #
    # def run_planner(self, stim_item):
    #     stim_ke_dict = {}
    #     stim_stab_dict = {}
    #
    #         platform_key_list = stim_item.get_platform_keys_list()[1:-1]
    #         if len(platform_key_list) < 2:
    #             continue
    #
    #         for platform_1, platform_2 in zip(platform_key_list, platform_key_list[1:]):
    #             p1_type = "_".join(platform_1.split('_')[1:])
    #             p2_type = "_".join(platform_1.split('_')[1:])
    #
    #             platform_pair_name = p1_type + "_" + p2_type
    #
    #             stim_num = int(stim_item_name.split("/")[-1].split("_")[1])
    #             prob = platform_pair_instability_dict.get(platform_pair_name, 0)
    #             if stim_num == 16:
    #                 prob = 3.0
    #             if stim_num == 21:
    #                 prob = 2.0
    #             if stim_num == 46:
    #                 prob = 2.0
    #             if stim_num == 36:
    #                 prob = 1.0
    #
    #             multiplier = np.ceil(ComputeUtils.sample_trunc_normal(prob, 0.0, 100.0, 0.5 * prob + 0.5))
    #             if prob == 0.0:
    #                 if "long" in p2_type or "wide" in p2_type or "top" in p2_type:
    #                     multiplier = np.ceil(ComputeUtils.sample_trunc_normal(5.0, 0.0, 30.0, 0.5 * 5.0 + 0.5))
    #                 else:
    #                     multiplier = 1.0
    #
    #             stim_ke_dict[stim_item_name] += multiplier * platform_pair_effort_dict.get(platform_pair_name, 0)
    #             stim_stab_dict[stim_item_name] += prob
    #
    #     z_ke_score = ComputeUtils.zscore_list(list(stim_ke_dict.values()))
    #     z_stab_score = ComputeUtils.zscore_list(list(stim_stab_dict.values()))
    #
    #     out_data_filepath = PathUtils.join(PathUtils.get_out_dirpath(), "model.csv")
    #     FileUtils.create_file(out_data_filepath)
    #     for key, ke, z_ke, stab, z_stab in zip(stim_ke_dict.keys(), stim_ke_dict.values(), z_ke_score,
    #                                            stim_stab_dict.values(), z_stab_score):
    #         t_stim_num = int(key.split("/")[-1].split("_")[1])
    #
    #         t_type_num = -1
    #         if t_stim_num == 5:  # longest path
    #             t_type_num = 0
    #         elif t_stim_num % 5 == 2 or t_stim_num % 5 == 3 or t_stim_num % 5 == 4:  # similar triplets
    #             t_type_num = t_stim_num // 5 * 2 + 1
    #         elif t_stim_num > 1 and t_stim_num % 5 == 1:  # triplets of the same platform
    #             t_type_num = t_stim_num // 5 * 2
    #
    #         FileUtils.write_row_to_file(out_data_filepath, [t_stim_num, str(t_type_num), ke, z_ke, stab, z_stab])
