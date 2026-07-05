from __future__ import annotations

import copy
import crocoddyl
import numpy as np
import time

from collections import deque, defaultdict
from enum import Enum
from typing import List

from mlr.share.projects.navigation.model.tasks.jump import JumpTask
from mlr.share.projects.navigation.model.tasks.turn import TurnTask
from mlr.share.projects.navigation.model.tasks.walk import WalkTask
from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import NavConfig
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.mujoco_utils import MujocoUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavTask
from mlr.share.projects.navigation.utils.platform_utils import Platform, PlatformType
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class NavOut(Enum):
    STIMULUS_NAME = 0
    STATUS = 1
    PATH_AS_STR = 2
    PATH_SYM_LEN = 3
    PATH_COST_KE = 4
    PATH_COST_CROCODDYL = 5


class NavStatus:
    NAV_SUCCESS = "NAV_SUCCESS"
    NAV_DYN_VAL_FAILURE = "NAV_DYNAMICS_VALIDATION_FAILURE"
    NAV_DYN_SIM_FAILURE = "NAV_DYNAMICS_FAILURE"


class NavState:
    def __init__(self, agent: NavAgent, scene: Stimulus, move_num, variation_num):
        self._agent = agent
        self._scene = scene

        self._move_num = move_num
        self._variation_num = variation_num

        self._ref_platform_name = None

        self._nav_status = None
        self._nav_task_type = []
        self._nav_tasks_list = []
        self._children_states_list = []

    def set_move_num(self, move_num):
        self._move_num = move_num

    def set_variation_num(self, variation_num):
        self._variation_num = variation_num

    def set_ref_platform_name(self, ref_platform_name):
        self._ref_platform_name = ref_platform_name
        self.get_scene().get_platform(ref_platform_name).set_visited(True)

    def set_nav_status(self, nav_status):
        self._nav_status = nav_status

    def set_nav_task_type(self, nav_task_type):
        self._nav_task_type = nav_task_type

    def is_done(self):
        for platform in self.get_scene().get_platforms_list():
            if platform.is_goal() and platform.has_been_visited():
                return True
        return False

    def add_nav_task(self, nav_task):
        self._nav_tasks_list.append(nav_task)

    def add_child_state(self, child_state: NavState):
        self._children_states_list.append(child_state)

    def reset_children_states_list(self):
        self._children_states_list = []

    def get_agent(self):
        return self._agent

    def get_agent_pose(self):
        return self.get_agent().get_current_pose()

    def get_scene(self) -> Stimulus:
        return self._scene

    def get_move_num(self):
        return self._move_num

    def get_variation_num(self):
        return self._variation_num

    def get_ref_platform_name(self):
        return self._ref_platform_name

    def get_nav_status(self):
        return self._nav_status

    def get_nav_task_type(self):
        return self._nav_task_type

    def get_nav_task(self):
        return self._nav_tasks_list

    def get_children_states_list(self):
        return self._children_states_list

    def get_total_cost_ke(self):
        return sum([nav_task.get_task_cost_ke() for nav_task in self.get_nav_task()])

    def get_total_cost_crocoddyl(self):
        return sum([task.get_task_cost_crocoddyl() for task in self.get_nav_task()])

    def get_candidate_platform_names_list(self):
        candidate_platform_names_list = []

        for platform_name in self.get_scene().get_platform_names_list():
            if self.get_scene().get_platform(platform_name).has_been_visited():
                continue

            if not self.get_scene().is_platform_progressive(platform_name, self.get_agent_pose()):
                continue

            gap = self.get_scene().get_gap_between_platform_edges(self.get_ref_platform_name(), platform_name)
            if gap > max(NavConfig.MAX_STEP_LENGTH, NavConfig.MAX_JUMP_LENGTH):
                continue

            candidate_platform_names_list.append(platform_name)

        return candidate_platform_names_list

    def get_ref_task_items(self):
        return {self.get_ref_platform_name(): [NavTask.WALK]}.items()

    def get_candidate_task_items(self, add_walk=False, add_jump=False):
        candidate_task_dict = defaultdict(list)
        for candidate_platform_name in self.get_candidate_platform_names_list():
            if add_walk:
                candidate_task_dict[candidate_platform_name].append(NavTask.WALK)
            if add_jump:
                candidate_task_dict[candidate_platform_name].append(NavTask.JUMP)
        return candidate_task_dict.items()


class NavModel:
    def __init__(self, nav_agent_name, nav_scene: Stimulus):
        self._agent = NavAgent(nav_agent_name)

        self._scene = nav_scene
        self._scene.load_stimuli_from_library()

        self._root_nav_state: NavState = NavState(self.get_agent(), self.get_scene(), 1, 1)

        self._nav_states_queue = deque([])

    @staticmethod
    def skew(mu, sigma, alpha, bound_min, bound_max):
        return ComputeUtils.sample_skew_normal(mu, sigma, alpha, bound_min, bound_max)

    @staticmethod
    def _get_walk_on_platform_tasks(nav_state, platform_name, agent_pos_x):
        dist = nav_state.get_scene().get_x_to_platform_edge(platform_name, agent_pos_x)[1]
        dist -= NavConfig.MIN_PLATFORM_PADDING

        if dist < 1.5 * NavConfig.MAX_STEP_LENGTH:
            return []

        dist = NavModel.skew(dist, NavConfig.SMALL_SKEW_SIGMA, -1, 0, dist)

        step_num = int(np.ceil(dist / NavConfig.MAX_STEP_LENGTH - 0.5))
        step_len = dist / (step_num + 0.5)

        tasks_list: List[NavTask] = [TurnTask(0)]
        for i in range(1, step_num + 1):
            tasks_list.append(WalkTask(step_length=step_len, is_first=i == 1, is_close=i == step_num))
        return tasks_list

    @staticmethod
    def compute_walk_tasks(nav_state: NavState, start_platform_name, final_platform_name) -> List[NavTask]:
        if start_platform_name == final_platform_name:
            return NavModel._get_walk_on_platform_tasks(nav_state, start_platform_name, nav_state.get_agent_pose()[0])

        gap = nav_state.get_scene().get_gap_between_platform_edges(start_platform_name, final_platform_name)
        if gap > NavConfig.MAX_STEP_LENGTH:
            return []

        walk_vec = nav_state.get_scene().get_point_closest_to_platform(start_platform_name, final_platform_name)
        walk_vec -= nav_state.get_agent_pose()[:2]
        if -25. < np.rad2deg(np.arctan2(walk_vec[1], walk_vec[0])) < 25.:
            walk_vec -= (walk_vec / (np.linalg.norm(walk_vec) + 1e-12)) * NavConfig.MIN_PLATFORM_PADDING
        else:  # compensate for the beveled corners
            walk_vec -= (walk_vec / (np.linalg.norm(walk_vec) + 1e-12)) * NavConfig.MIN_PLATFORM_PADDING * 2

        goal_vec = walk_vec[0] + nav_state.get_agent_pose()[0] + NavConfig.MAX_STEP_LENGTH

        step_num = int(np.ceil(np.linalg.norm(walk_vec) / NavConfig.MAX_STEP_LENGTH - 0.5))
        step_len = np.linalg.norm(walk_vec) / (step_num + 0.5)

        tasks_list: List[NavTask] = [TurnTask(np.arctan2(walk_vec[1], walk_vec[0]))]
        for i in range(step_num + 1):
            tasks_list.append(WalkTask(step_length=step_len, is_first=i == 0, is_close=i == step_num))

        tasks_list.append(TurnTask(0))
        tasks_list.append(WalkTask(step_length=NavConfig.MAX_STEP_LENGTH, is_lunge=True))

        tasks_list += NavModel._get_walk_on_platform_tasks(nav_state, final_platform_name, goal_vec)

        return tasks_list

    @staticmethod
    def compute_jump_tasks(nav_state: NavState, final_platform_name) -> List[NavTask]:
        if nav_state.get_ref_platform_name() == final_platform_name:
            return []

        min_dist = nav_state.get_scene().get_x_to_platform_edge(final_platform_name, nav_state.get_agent_pose()[0])[0]
        if min_dist >= NavConfig.MAX_JUMP_LENGTH - NavConfig.MIN_PLATFORM_PADDING:
            return []

        surface_xy = Platform.get_platform_top_surface_xy(PlatformType.get_base())

        skewed_x_bound = surface_xy[0] / 2 - NavConfig.MIN_PLATFORM_PADDING
        skewed_y_bound = surface_xy[1] / 6

        skewed_x = NavModel.skew(0, NavConfig.LARGE_SKEW_SIGMA, -1, -skewed_x_bound, skewed_x_bound)
        skewed_y = NavModel.skew(0, NavConfig.SMALL_SKEW_SIGMA, 1, -skewed_y_bound, skewed_y_bound)
        skewed_y = abs(skewed_y)

        jump_vec = nav_state.get_scene().get_platform_center(final_platform_name)
        jump_vec[2] = 0.0
        jump_vec[0] += skewed_x
        jump_vec[:2] -= nav_state.get_agent_pose()[:2]

        if not (-15.0 < np.rad2deg(np.arctan2(jump_vec[1], jump_vec[0])) < 15.0):
            if jump_vec[1] > 0:
                jump_vec[1] -= skewed_y
            else:
                jump_vec[1] += skewed_y

        if np.linalg.norm(jump_vec) > NavConfig.MAX_JUMP_LENGTH:
            jump_vec = (jump_vec / (np.linalg.norm(jump_vec) + 1e-12)) * NavConfig.MAX_JUMP_LENGTH
            return [TurnTask(np.arctan2(jump_vec[1], jump_vec[0])), JumpTask(jump_vec)]

        return [TurnTask(np.arctan2(jump_vec[1], jump_vec[0])), JumpTask(jump_vec)]

    @staticmethod
    def run_kinematics(nav_task_type, nav_state: NavState, final_platform_name):
        if nav_task_type == NavTask.WALK:
            tasks_list = NavModel.compute_walk_tasks(nav_state, nav_state.get_ref_platform_name(), final_platform_name)
        elif nav_task_type == NavTask.JUMP:
            tasks_list = NavModel.compute_jump_tasks(nav_state, final_platform_name)
        else:
            return Msg.FAILURE

        if len(tasks_list) == 0:
            return Msg.FAILURE

        for task in tasks_list:
            task.run_task(nav_state.get_agent())
            task.tally_task(nav_state.get_ref_platform_name(), final_platform_name)
            nav_state.add_nav_task(task)

        if NavConfig.VIEW_KINEMATICS:
            display = crocoddyl.MeshcatDisplay(NavAgent.get_robot())
            display.rate = -1
            display.freq = 1
            start_time = time.time()
            while True:
                if time.time() - start_time >= 5.0:
                    break
                for task in tasks_list:
                    task_solver = task.get_task_solver()
                    if task_solver is not None:
                        display.displayFromSolver(task_solver)
                time.sleep(0.01)

        return Msg.SUCCESS

    @staticmethod
    def run_dynamics_validator(nav_state: NavState):
        task_registry_list = []
        for nav_task in nav_state.get_nav_task():
            task_registry_list.extend(nav_task.get_task_registry_list())

        for task_registry in task_registry_list:
            if task_registry.get_force_left() is not None:
                left_pos = task_registry.get_force_left_pos()
                left_loc = task_registry.get_platform_name_left()
                if not nav_state.get_scene().is_within_bounds(left_loc, left_pos):
                    return Msg.FAILURE

            if task_registry.get_force_right() is not None:
                right_pos = task_registry.get_force_right_pos()
                right_loc = task_registry.get_platform_name_right()
                if not nav_state.get_scene().is_within_bounds(right_loc, right_pos):
                    return Msg.FAILURE

        return Msg.SUCCESS

    @staticmethod
    def run_dynamics_simulator(nav_state: NavState):
        mujoco_utils = MujocoUtils(nav_state.get_scene().get_stimulus_mjcf_filepath())

        task_registry_list = []
        for nav_task in nav_state.get_nav_task():
            task_registry_list.extend(nav_task.get_task_registry_list())

        if NavConfig.DEBUG_DYNAMICS:
            mujoco_utils.visualize(task_registry_list)

        status = mujoco_utils.simulate(task_registry_list)

        return status

    def init_planner(self):
        self._root_nav_state.set_nav_status(NavStatus.NAV_SUCCESS)
        self._root_nav_state.set_ref_platform_name(self.get_scene().get_platform_names_list()[0])
        self._nav_states_queue.append(self._root_nav_state)

    def run_planner(self):
        self.init_planner()

        while len(self._nav_states_queue) > 0:
            nav_state = self._nav_states_queue.popleft()

            if nav_state.is_done():
                continue

            if len(nav_state.get_candidate_platform_names_list()) == 0:
                continue

            if nav_state.get_move_num() > NavConfig.MAX_TASKS_PER_PLAN:
                continue

            variation_num = 0
            next_tasks_queue = deque(nav_state.get_ref_task_items())

            while len(next_tasks_queue) > 0:

                next_platform_name, nav_task_type_list = next_tasks_queue.popleft()

                for nav_task_type in nav_task_type_list:
                    next_nav_state: NavState = copy.deepcopy(nav_state)

                    kin_status = NavModel.run_kinematics(nav_task_type, next_nav_state, next_platform_name)
                    if kin_status == Msg.FAILURE:
                        if nav_task_type == NavTask.WALK:
                            if nav_state.get_ref_platform_name() == next_platform_name:
                                next_tasks_queue.extend(next_nav_state.get_candidate_task_items(add_walk=True))
                            else:
                                next_tasks_queue.extend({next_platform_name: [NavTask.JUMP]}.items())
                        continue

                    dyn_val_status = NavModel.run_dynamics_validator(next_nav_state)
                    if dyn_val_status == Msg.FAILURE:
                        Msg.print_info(f"INFO [NavModel]: failed dynamics validator")
                        next_nav_state.set_nav_status(NavStatus.NAV_DYN_VAL_FAILURE)
                        next_nav_state.get_agent().update_current_pose(nav_state.get_agent_pose())
                    else:
                        dyn_sim_status = NavModel.run_dynamics_simulator(next_nav_state)
                        if dyn_sim_status == Msg.FAILURE:
                            Msg.print_info(f"INFO [NavModel]: failed dynamics simulator")
                            next_nav_state.set_nav_status(NavStatus.NAV_DYN_SIM_FAILURE)
                            next_nav_state.get_agent().update_current_pose(nav_state.get_agent_pose())
                        else:
                            Msg.print_info(f"INFO [NavModel]: planner move successful!")
                            next_nav_state.set_nav_status(NavStatus.NAV_SUCCESS)
                            next_nav_state.set_ref_platform_name(next_platform_name)

                    variation_num += 1
                    next_nav_state.set_move_num(nav_state.get_move_num() + 1)
                    next_nav_state.set_variation_num(variation_num)
                    next_nav_state.set_nav_task_type(nav_task_type)
                    next_nav_state.reset_children_states_list()
                    nav_state.add_child_state(next_nav_state)
                    self._nav_states_queue.append(next_nav_state)

    def print_possible_paths(self):
        def _explore_state(nav_state: NavState, input_path_str=""):
            shape = nav_state.get_ref_platform_name().split("_")[0]
            platform_number = nav_state.get_ref_platform_name().split("_")[-1]

            path_str = input_path_str + f" --{nav_state.get_nav_task_type()[0]}-> [{shape}{platform_number}]"

            if nav_state.is_done() or nav_state.get_move_num() ==  NavConfig.MAX_TASKS_PER_PLAN:
                print(f"root1{path_str}")
                return

            for child in nav_state.get_children_states_list():
                _explore_state(child, path_str)

        for child_state in self._root_nav_state.get_children_states_list():
            _explore_state(child_state)

    def save_state_to_file(self, out_filepath):
        FileUtils.create_file(out_filepath)
        FileUtils.write_row_to_file(out_filepath, [NavOut.STIMULUS_NAME.name,
                                                   NavOut.STATUS.name,
                                                   NavOut.PATH_AS_STR.name,
                                                   NavOut.PATH_SYM_LEN.name,
                                                   NavOut.PATH_COST_KE.name,
                                                   NavOut.PATH_COST_CROCODDYL.name])

        def _explore_state(nav_state: NavState, input_path_str=""):
            shape = nav_state.get_ref_platform_name().split("_")[0]
            platform_number = nav_state.get_ref_platform_name().split("_")[-1]

            path_str = input_path_str + f" --{nav_state.get_nav_task_type()[0]}-> [{shape}{platform_number}]"
            symbolic_len = path_str.count("->")

            if nav_state.is_done():
                FileUtils.write_row_to_file(out_filepath, [nav_state.get_scene().get_stimulus_name(),
                                                           nav_state.get_nav_status().lower(),
                                                           f"root1{path_str}",
                                                           symbolic_len,
                                                           nav_state.get_total_cost_ke(),
                                                           nav_state.get_total_cost_crocoddyl()])
                return

            for child in nav_state.get_children_states_list():
                _explore_state(child, path_str)

        for child_state in self._root_nav_state.get_children_states_list():
            _explore_state(child_state)

    def get_scene(self):
        return self._scene

    def get_agent(self):
        return self._agent
