import ast
import click
import csv
import multiprocessing
import numpy as np
import time

from itertools import repeat

from mlr.share.projects.block_building.model.exp import ExpType
from mlr.share.projects.block_building.model.genea.exp import GeneaExperiment
from mlr.share.projects.block_building.model.vlm.exp import VLMExperiment
from mlr.share.projects.block_building.run_risk_test import run_risk_test
from mlr.share.projects.block_building.utils.core_utils import ConfigUtils
from mlr.share.projects.block_building.utils.path_utils import PathUtils


class Trials:
    def __init__(self, exp_type, trial_name, trial_first_moves, trial_move_num):
        self._exp_type = exp_type
        self._trial_name = trial_name
        self._trial_first_moves = trial_first_moves
        self._trial_move_num = trial_move_num

    def get_exp_type(self):
        return self._exp_type

    def get_trial_name(self):
        return self._trial_name

    def get_trial_first_moves(self):
        return self._trial_first_moves

    def get_trial_move_number(self):
        return self._trial_move_num


def get_trials_list(query_exp_type=None):
    return_trials_list = []

    trials_info_list_filename = PathUtils.join(PathUtils.get_exp_data_dirpath(), "trials_list.csv")
    with open(trials_info_list_filename, "r") as trials_info_list_file:
        trials_info_list = csv.reader(trials_info_list_file, delimiter=',')
        trials_info_list = list(trials_info_list)
        for trial_info in trials_info_list[1:]:
            exp_type = ExpType.get_exp_type(trial_info[0])
            if query_exp_type is not None:
                if not query_exp_type == exp_type:
                    continue

            trial_name = trial_info[1]
            trial_first_moves = ast.literal_eval(trial_info[2])
            if len(trial_first_moves) == 0:
                return_trials_list.append(Trials(exp_type, trial_name, None, -1))

            for move_number, trial_move in enumerate(trial_first_moves, 1):
                return_trials_list.append(Trials(exp_type, trial_name, trial_move, move_number))

    return return_trials_list


def run_exp_trials(exp_type, max_steps_per_plan=ConfigUtils.DEFAULT_MAX_STEPS_PER_PLAN, total_runs=1):
    trails_list = get_trials_list(exp_type)

    core_identifiers_list = {}

    for trial in trails_list:
        for _ in range(total_runs):
            core_identifier = trial.get_exp_type() + PathUtils.UNDERSCORE + trial.get_trial_name()

            if core_identifier in core_identifiers_list.keys():
                core_identifiers_list[core_identifier] += 1
            else:
                core_identifiers_list[core_identifier] = 1

            folder_counter = core_identifiers_list[core_identifier]

            experiment = GeneaExperiment(exp_type=trial.get_exp_type(), exp_trial_num=trial.get_trial_name(),
                                         exp_trial_move_num=trial.get_trial_move_number(),
                                         folder_counter=folder_counter, max_steps_per_plan=max_steps_per_plan)
            experiment.run_experiment(first_block_names_to_grab=trial.get_trial_first_moves())
            core_identifiers_list[core_identifier] = experiment.get_folder_counter_value()


def run_trial(trial, folder_counter, max_steps_per_plan=ConfigUtils.DEFAULT_MAX_STEPS_PER_PLAN):
    experiment = GeneaExperiment(exp_type=trial.get_exp_type(), exp_trial_num=trial.get_trial_name(),
                                 exp_trial_move_num=trial.get_trial_move_number(),
                                 folder_counter=folder_counter, max_steps_per_plan=max_steps_per_plan)
    experiment.run_experiment(first_block_names_to_grab=trial.get_trial_first_moves())


def run_exp_trial_multiprocess(exp_type, max_steps_per_plan=ConfigUtils.DEFAULT_MAX_STEPS_PER_PLAN, total_runs=1):
    trails_list = get_trials_list(exp_type)

    for trial_num in range(1, total_runs + 1):
        cores = multiprocessing.cpu_count()
        pool = multiprocessing.Pool(processes=cores, initializer=np.random.seed(int(time.time())))
        pool.starmap(run_trial, zip(trails_list, repeat(trial_num), repeat(max_steps_per_plan)))


def run_all_risk_test(exp_type, total_runs=1):
    trails_list = get_trials_list(exp_type)

    seed_list = np.arange(len(trails_list)) * 1000

    cores = multiprocessing.cpu_count()
    pool = multiprocessing.Pool(processes=cores)
    pool.starmap(run_risk_test, zip([t.get_trial_name() for t in trails_list],
                                    repeat(exp_type),
                                    repeat(total_runs),
                                    seed_list))


def run_vlm_experiments(exp_type):
    experiment = VLMExperiment(exp_type=exp_type, exp_trials_list=get_trials_list(exp_type))
    experiment.run_experiment()


@click.command()
@click.option("-n", "--num_runs", default="1", type=click.STRING, help="total number of runs per trial")
@click.option("-r", "--run_risk", default=False, is_flag=True, help="run risk test only")
@click.option("-a", "--action_inference", default=False, is_flag=True, help="run action inference trials")
@click.option("-g", "--goal_inference", default=False, is_flag=True, help="run goal inference trials")
@click.option("-e", "--effector_inference", default=False, is_flag=True, help="run number of hands trials")
@click.option("-d", "--difficulty", default=False, is_flag=True, help="run action inference trials")
@click.option("-v", "--vlm", default=False, is_flag=True, help="run vlm experiment")
def main(num_runs, run_risk, action_inference, goal_inference, effector_inference, difficulty, vlm):
    if run_risk:
        run_all_risk_test(ExpType.DIFFICULTY, total_runs=int(num_runs))
        return

    if action_inference:
        run_exp_trials(ExpType.ACTION, total_runs=int(num_runs))
        return

    if goal_inference:
        run_exp_trials(ExpType.GOAL, total_runs=int(num_runs))
        return

    if effector_inference:
        run_exp_trials(ExpType.HAND, total_runs=int(num_runs))
        return

    if difficulty:
        run_exp_trial_multiprocess(ExpType.DIFFICULTY, ConfigUtils.DIFF_MAX_STEPS_PER_PLAN, total_runs=int(num_runs))
        return

    if vlm:
        run_vlm_experiments(ExpType.DIFFICULTY)
        run_vlm_experiments(ExpType.ACTION)
        run_vlm_experiments(ExpType.GOAL)
        run_vlm_experiments(ExpType.HAND)
        return


if __name__ == '__main__':
    main()
