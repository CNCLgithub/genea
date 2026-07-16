from __future__ import annotations

import copy
import crocoddyl
import numpy as np
import time

from collections import deque
from enum import Enum
from typing import List

from mlr.share.projects.navigation.model.tasks.jump import JumpTask
from mlr.share.projects.navigation.model.tasks.turn import TurnTask
from mlr.share.projects.navigation.model.tasks.walk import WalkTask
from mlr.share.projects.navigation.utils.agent_utils import NavAgent
from mlr.share.projects.navigation.utils.compute_utils import ComputeUtils
from mlr.share.projects.navigation.utils.config_utils import NavConfig, CoreConfig
from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.msg_utils import Msg
from mlr.share.projects.navigation.utils.mujoco_utils import MujocoUtils
from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavTaskRegistry
from mlr.share.projects.navigation.utils.platform_utils import Platform, PlatformType
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class NavOutPlanner(Enum):
    NAME = 0
    SUCCESS = 1
    RUN_NUM = 2
    PATH_COST_KE = 3
    PATH_COST_CROCODDYL = 4
    PATH_SYM_LEN = 5
    PATH_ATTEMPTS = 6
    PATH_AS_STR = 7


class NavOutHeuristic(Enum):
    NAME = 0
    COUNT = 1
    P1 = 2
    P2 = 3
    P3 = 4
    P4 = 5
    P5 = 6
    P6 = 7
    P7 = 8
    P8 = 9
    P9 = 10


class NavMove:
    def __init__(self, source_platform_name, target_platform_name, nav_task_type=NavTask.WALK):
        self._source_platform_name = source_platform_name
        self._target_platform_name = target_platform_name
        self._nav_task_type = nav_task_type

        self._attempts = 1

        self._cost_ke = 0.0
        self._cost_crocoddyl = 0.0

    def set_attempts(self, attempts):
        self._attempts = attempts

    def add_cost_ke(self, cost_ke):
        self._cost_ke += cost_ke

    def add_cost_crocoddyl(self, cost_crocoddyl):
        self._cost_crocoddyl += cost_crocoddyl

    def jump(self):
        if not self.get_source_platform_name() == self.get_target_platform_name():
            return [NavMove(self.get_source_platform_name(), self.get_target_platform_name(), NavTask.JUMP)]
        return []

    def redo(self):
        nav_move = NavMove(self.get_source_platform_name(), self.get_target_platform_name(), self.get_nav_task_type())
        nav_move.set_attempts(self.get_attempts() + 1)
        nav_move.add_cost_ke(self.get_cost_ke())
        nav_move.add_cost_crocoddyl(self.get_cost_crocoddyl())
        return nav_move

    def get_source_platform_name(self):
        return self._source_platform_name

    def get_target_platform_name(self):
        return self._target_platform_name

    def get_nav_task_type(self):
        return self._nav_task_type

    def get_attempts(self):
        return self._attempts

    def get_cost_ke(self):
        return self._cost_ke

    def get_cost_crocoddyl(self):
        return self._cost_crocoddyl


class NavState:
    def __init__(self, agent: NavAgent, scene: Stimulus, move_num, variation_num):
        self._agent = agent
        self._scene = scene

        self._move_num = move_num
        self._variation_num = variation_num

        self._ref_platform_name = None

        self._nav_task_type = None
        self._nav_tasks_list = []

        self._nav_total_cost_ke = 0.0
        self._nav_total_cost_crocoddyl = 0.0
        self._nav_total_attempts = 0

        self._children_states_list = []

    def set_move_num(self, move_num):
        self._move_num = move_num

    def set_variation_num(self, variation_num):
        self._variation_num = variation_num

    def set_ref_platform_name(self, ref_platform_name):
        self._ref_platform_name = ref_platform_name
        self.get_scene().get_platform(ref_platform_name).set_visited(True)

    def set_nav_task_type(self, nav_task_type):
        self._nav_task_type = nav_task_type

    def set_nav_total_cost_ke(self, nav_cost_ke):
        self._nav_total_cost_ke = nav_cost_ke

    def set_nav_total_cost_crocoddyl(self, nav_cost_crocoddyl):
        self._nav_total_cost_crocoddyl = nav_cost_crocoddyl

    def set_total_attempts(self, attempts):
        self._nav_total_attempts = attempts

    def is_success(self):
        for platform in self.get_scene().get_platforms_list():
            if platform.is_goal() and platform.has_been_visited():
                return True
        return False

    def is_done(self):
        condition1 = self.get_move_num() == CoreConfig.EXP_MAX_MOVE_COUNT
        condition2 = self.get_total_attempts() >= CoreConfig.EXP_MAX_MOVE_ATTEMPTS
        return condition1 or condition2 or self.is_success()

    def add_nav_task(self, nav_task):
        self._nav_tasks_list.append(nav_task)

    def add_child_state(self, child_state: NavState):
        self._children_states_list.append(child_state)

    def reset(self):
        self._children_states_list = []
        self._nav_tasks_list = []
        self._nav_total_cost_ke = 0.0
        self._nav_total_cost_crocoddyl = 0.0

    def compute_task_cost_ke(self):
        return sum([nav_task.get_task_cost_ke() for nav_task in self.get_nav_task()])

    def compute_task_cost_crocoddyl(self):
        return sum([task.get_task_cost_crocoddyl() for task in self.get_nav_task()])

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

    def get_nav_task_type(self):
        return self._nav_task_type

    def get_nav_task(self):
        return self._nav_tasks_list

    def get_children_states_list(self):
        return self._children_states_list

    def get_total_cost_ke(self):
        return self._nav_total_cost_ke

    def get_total_cost_crocoddyl(self):
        return self._nav_total_cost_crocoddyl

    def get_total_attempts(self):
        return self._nav_total_attempts

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

    def get_candidate_moves(self) -> List[NavMove]:
        candidate_moves_list = [NavMove(self._ref_platform_name, self._ref_platform_name)]
        for candidate_platform_name in self.get_candidate_platform_names_list():
            candidate_moves_list.append(NavMove(self._ref_platform_name, candidate_platform_name))
        return candidate_moves_list


class NavModel:
    def __init__(self, nav_agent_name, nav_scene: Stimulus):
        self._agent = NavAgent(nav_agent_name)

        self._scene = nav_scene
        self._scene.load_stimuli_from_library()

        self._root_nav_state: NavState = NavState(self.get_agent(), self.get_scene(), 1, 1)

        self._nav_states_queue = deque([])

    @staticmethod
    def _skew(mu, sigma, alpha, bound_min, bound_max):
        return ComputeUtils.sample_skew_normal(mu, sigma, alpha, bound_min, bound_max)

    @staticmethod
    def _correct_walk(walk_vec_xy):  # compensate for the beveled corners
        if -25. < np.rad2deg(np.arctan2(walk_vec_xy[1], walk_vec_xy[0])) < 25.:
            walk_vec_xy -= (walk_vec_xy / (np.linalg.norm(walk_vec_xy) + 1e-12)) * NavConfig.MIN_PLATFORM_PADDING
        elif -47.5 < np.rad2deg(np.arctan2(walk_vec_xy[1], walk_vec_xy[0])) < 47.5:
            walk_vec_xy -= (walk_vec_xy / (np.linalg.norm(walk_vec_xy) + 1e-12)) * NavConfig.MIN_PLATFORM_PADDING * 2.5
        else:
            walk_vec_xy -= (walk_vec_xy / (np.linalg.norm(walk_vec_xy) + 1e-12)) * NavConfig.MIN_PLATFORM_PADDING * 1.25
        return walk_vec_xy

    @staticmethod
    def _compute_walk(walk_vec_xy, do_skew=False, is_other=False):
        walk_disp_xy = np.linalg.norm(walk_vec_xy)

        if do_skew:
            walk_disp_xy = NavModel._skew(walk_disp_xy, NavConfig.WALK_SKEW_SIGMA, -1, 0, walk_disp_xy)

        step_num = int(np.ceil(walk_disp_xy / NavConfig.MAX_STEP_LENGTH - 0.5))
        step_len = walk_disp_xy / (step_num + 0.5)

        task_list: List[NavTask] = [TurnTask(np.arctan2(walk_vec_xy[1], walk_vec_xy[0]))]
        for i in range(step_num + 1):
            task_list.append(WalkTask(step_length=step_len, is_first=i == 0, is_close=i == step_num, is_other=is_other))
        return task_list

    @staticmethod
    def compute_walk_tasks(nav_state: NavState, start_platform_name, final_platform_name) -> List[NavTask]:
        if start_platform_name == final_platform_name:
            dist = nav_state.get_scene().get_space_to_platform_edge(start_platform_name, nav_state.get_agent_pose()[:2])
            walk_vec_xy = np.array([1., 0]) * dist
            walk_vec_xy = NavModel._correct_walk(walk_vec_xy)

            if np.linalg.norm(walk_vec_xy) < 2.25 * NavConfig.MAX_STEP_LENGTH:
                return []
            return NavModel._compute_walk(walk_vec_xy, do_skew=True)

        gap = nav_state.get_scene().get_gap_between_platform_edges(start_platform_name, final_platform_name)
        if gap > NavConfig.MAX_STEP_LENGTH:
            return []

        walk_vec_xy = nav_state.get_scene().get_vec_closest_to_goal_platform(start_platform_name, final_platform_name)
        walk_vec_xy -= nav_state.get_agent_pose()[:2]
        walk_vec_xy = NavModel._correct_walk(walk_vec_xy)
        task_list = NavModel._compute_walk(walk_vec_xy, do_skew=False)

        agent_xy = nav_state.get_agent_pose()[:2] + walk_vec_xy
        walk_vec_xy = nav_state.get_scene().get_vec_to_next_platform(agent_xy, final_platform_name)
        task_list.append(TurnTask(np.arctan2(walk_vec_xy[1], walk_vec_xy[0])))
        task_list.append(WalkTask(step_length=NavConfig.MAX_STEP_LENGTH, is_lunge=True))

        walk_vec_xy = agent_xy + (walk_vec_xy / (np.linalg.norm(walk_vec_xy) + 1e-12)) * NavConfig.MAX_STEP_LENGTH
        walk_vec_xy = nav_state.get_scene().get_platform_vantage_right(final_platform_name)[:2] - walk_vec_xy
        task_list += NavModel._compute_walk(walk_vec_xy, do_skew=False, is_other=True)

        return task_list

    @staticmethod
    def compute_jump_tasks(nav_state: NavState, final_platform_name) -> List[NavTask]:
        if nav_state.get_ref_platform_name() == final_platform_name:
            return []

        min_dist = nav_state.get_scene().get_x_to_platform_edge(final_platform_name, nav_state.get_agent_pose()[0])[0]
        if min_dist >= NavConfig.MAX_JUMP_LENGTH - NavConfig.MIN_PLATFORM_PADDING:
            return []

        surface_xy = Platform.get_platform_top_surface_xy(PlatformType.get_base())

        bound = surface_xy[0] / 2 - NavConfig.MIN_PLATFORM_PADDING

        skewed_x = NavModel._skew(-surface_xy[0] / 4, NavConfig.JUMP_SKEW_SIGMA, -1, -bound, bound)
        skewed_y = NavModel._skew(+surface_xy[1] / 6, NavConfig.JUMP_SKEW_SIGMA, +0, -bound, bound)
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
    def run_kinematics(nav_task_type, nav_state: NavState, start_platform_name, final_platform_name):
        if nav_task_type == NavTask.WALK:
            tasks_list = NavModel.compute_walk_tasks(nav_state, start_platform_name, final_platform_name)
        elif nav_task_type == NavTask.JUMP:
            tasks_list = NavModel.compute_jump_tasks(nav_state, final_platform_name)
        else:
            return Msg.FAILURE

        if len(tasks_list) == 0:
            return Msg.FAILURE

        for task in tasks_list:
            task.run_task(nav_state.get_agent())
            task.tally_task(start_platform_name, final_platform_name)
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
        if not CoreConfig.RUN_DYNAMICS:
            return Msg.SUCCESS

        task_registry_list = []
        for nav_task in nav_state.get_nav_task():
            task_registry_list.extend(nav_task.get_task_registry_list())

        for task_index, task_registry in enumerate(task_registry_list):
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
        if not CoreConfig.RUN_DYNAMICS:
            return Msg.SUCCESS

        mujoco_utils = MujocoUtils(nav_state.get_scene().get_stimulus_mjcf_filepath())

        task_registry_list = []
        for nav_task in nav_state.get_nav_task():
            task_registry_list.extend(nav_task.get_task_registry_list())

        if NavConfig.DEBUG_DYNAMICS:
            mujoco_utils.visualize(task_registry_list)

        status = mujoco_utils.simulate(task_registry_list)

        return status

    def init_planner(self):
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

            Msg.print_info(f"{nav_state.get_move_num()}_{nav_state.get_variation_num()} {len(self._nav_states_queue)}")

            variation_num = 0
            next_moves_list: List[NavMove] = nav_state.get_candidate_moves()
            while len(next_moves_list) > 0:
                next_move = next_moves_list.pop(0)
                src_platform = next_move.get_source_platform_name()
                dst_platform = next_move.get_target_platform_name()
                nav_task_type = next_move.get_nav_task_type()

                Msg.print_info(f"{src_platform} -- {nav_task_type} -> {dst_platform}")

                next_nav_state: NavState = copy.deepcopy(nav_state)
                next_nav_state.reset()

                kin_status = NavModel.run_kinematics(nav_task_type, next_nav_state, src_platform, dst_platform)
                if kin_status == Msg.FAILURE:
                    Msg.print_info(f"INFO [NavModel]: failed kinematics")
                    if nav_task_type == NavTask.WALK:
                        next_moves_list.extend(next_move.jump())
                    continue

                next_move.add_cost_ke(next_nav_state.compute_task_cost_ke())
                next_move.add_cost_crocoddyl(next_nav_state.compute_task_cost_crocoddyl())

                if src_platform == dst_platform:
                    next_moves_list = []

                dyn_val_status = NavModel.run_dynamics_validator(next_nav_state)
                if dyn_val_status == Msg.FAILURE:
                    Msg.print_info(f"INFO [NavModel]: failed dynamics validator")
                    if nav_task_type == NavTask.WALK:
                        next_moves_list.extend(next_move.jump())
                    elif nav_task_type == NavTask.JUMP and next_move.get_attempts() < CoreConfig.EXP_MAX_MOVE_ATTEMPTS:
                        next_moves_list.append(next_move.redo())
                    continue

                dyn_sim_status = NavModel.run_dynamics_simulator(next_nav_state)
                if dyn_sim_status == Msg.FAILURE:
                    Msg.print_info(f"INFO [NavModel]: failed dynamics simulator")
                    if next_move.get_attempts() < CoreConfig.EXP_MAX_MOVE_ATTEMPTS:
                        next_moves_list.append(next_move.redo())
                        continue

                Msg.print_info(f"INFO [NavModel]: SUCCESS!")

                variation_num += 1
                next_nav_state.set_move_num(nav_state.get_move_num() + 1)
                next_nav_state.set_variation_num(variation_num)
                next_nav_state.set_nav_task_type(nav_task_type)
                next_nav_state.set_ref_platform_name(dst_platform)

                next_nav_state.set_nav_total_cost_ke(next_move.get_cost_ke())
                next_nav_state.set_nav_total_cost_crocoddyl(next_move.get_cost_crocoddyl())
                next_nav_state.set_total_attempts(next_move.get_attempts())

                nav_state.add_child_state(next_nav_state)
                self._nav_states_queue.append(next_nav_state)

    def run_stability_heuristic(self, out_filepath):
        platform_instability_count_list = [0] * 9

        platform_names_list = self.get_scene().get_platform_names_list()
        platform_names_list = platform_names_list[1:-1]

        for platform_index, platform_name in enumerate(platform_names_list):
            for _ in range(CoreConfig.EXP_STABILITY_MOVES):
                task_registry_list = []
                for _ in range(CoreConfig.EXP_STABILITY_COUNT):
                    platform = self.get_scene().get_platform(platform_name)
                    platform_surface_xy = Platform.get_platform_top_surface_xy(platform.get_platform_type())

                    force_pos_vec = self.get_scene().get_platform_center(platform_name)
                    force_pos_vec[0] = np.random.uniform(-platform_surface_xy[0] / 2, platform_surface_xy[0] / 2)
                    force_pos_vec[1] = np.random.uniform(-platform_surface_xy[1] / 2, platform_surface_xy[1] / 2)

                    force_left = NavTask.get_random_force(force_pos_vec)
                    force_right = NavTask.get_random_force(force_pos_vec)

                    task_registry = NavTaskRegistry()
                    task_registry.set_platform_name_left(platform_name)
                    task_registry.set_platform_name_right(platform_name)
                    task_registry.set_force_left(force_left)
                    task_registry.set_force_right(force_right)

                    task_registry_list.append(task_registry)

                mujoco_utils = MujocoUtils(self.get_scene().get_stimulus_mjcf_filepath())
                sim_status = mujoco_utils.simulate(task_registry_list)

                platform_instability_count_list[platform_index] += int(sim_status == Msg.FAILURE)

        if not FileUtils.is_file(out_filepath):
            FileUtils.create_file(out_filepath)
            FileUtils.write_row_to_file(out_filepath, [NavOutHeuristic.NAME.name,
                                                       NavOutHeuristic.COUNT.name,
                                                       NavOutHeuristic.P1.name,
                                                       NavOutHeuristic.P2.name,
                                                       NavOutHeuristic.P3.name,
                                                       NavOutHeuristic.P4.name,
                                                       NavOutHeuristic.P5.name,
                                                       NavOutHeuristic.P6.name,
                                                       NavOutHeuristic.P7.name,
                                                       NavOutHeuristic.P8.name,
                                                       NavOutHeuristic.P9.name])

        FileUtils.write_row_to_file(out_filepath, [self.get_scene().get_stimulus_name(),
                                                   len(platform_names_list),
                                                   *platform_instability_count_list])

    def print_possible_paths(self):
        def _explore_state(nav_state: NavState, input_path_str=""):
            shape = nav_state.get_ref_platform_name().split("_")[0]
            platform_number = nav_state.get_ref_platform_name().split("_")[-1]

            path_str = input_path_str + f" --{nav_state.get_nav_task_type()[0]}-> [{shape}{platform_number}]"

            if nav_state.is_done():
                print(path_str)
                return

            for child in nav_state.get_children_states_list():
                _explore_state(child, path_str)

        for child_state in self._root_nav_state.get_children_states_list():
            _explore_state(child_state, "root1")

    def save_state_to_file(self, out_filepath, run_num):
        if not FileUtils.is_file(out_filepath):
            FileUtils.create_file(out_filepath)
            FileUtils.write_row_to_file(out_filepath, [NavOutPlanner.NAME.name,
                                                       NavOutPlanner.SUCCESS.name,
                                                       NavOutPlanner.RUN_NUM.name,
                                                       NavOutPlanner.PATH_COST_KE.name,
                                                       NavOutPlanner.PATH_COST_CROCODDYL.name,
                                                       NavOutPlanner.PATH_SYM_LEN.name,
                                                       NavOutPlanner.PATH_ATTEMPTS.name,
                                                       NavOutPlanner.PATH_AS_STR.name])

        def _explore_state(nav_state: NavState, input_path_as_str, total_attempts=0, total_ke=0.0, total_croc=0.0):
            shape = nav_state.get_ref_platform_name().split("_")[0]
            platform_number = nav_state.get_ref_platform_name().split("_")[-1]

            path_as_str = input_path_as_str + f" --{nav_state.get_nav_task_type()[0]}-> [{shape}{platform_number}]"
            attempts = total_attempts + nav_state.get_total_attempts()
            cost_ke = total_ke + nav_state.get_total_cost_ke()
            cost_croc = total_croc + nav_state.get_total_cost_crocoddyl()

            if nav_state.is_done():
                FileUtils.write_row_to_file(out_filepath, [nav_state.get_scene().get_stimulus_name(),
                                                           int(nav_state.is_success()),
                                                           str(run_num),
                                                           cost_ke,
                                                           cost_croc,
                                                           path_as_str.count("->"),
                                                           attempts,
                                                           path_as_str])
                return

            for child in nav_state.get_children_states_list():
                _explore_state(child, path_as_str, attempts, cost_ke, cost_croc)

        for child_state in self._root_nav_state.get_children_states_list():
            _explore_state(child_state, "root1")

    def get_scene(self):
        return self._scene

    def get_agent(self):
        return self._agent
