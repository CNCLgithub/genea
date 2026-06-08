from mlr.share.projects.navigation.model.planner import NavModel
from mlr.share.projects.navigation.model.stimuli import StimuliTestJump, StimuliTestWalk
from mlr.share.projects.navigation.utils.navigation_utils import NavAgent
from mlr.share.projects.navigation.utils.stimuli_utils import Stimulus


class Experiment:
    @staticmethod
    def run_jump_test():
        nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliTestJump()))
        nav_model.add_jump_task()
        nav_model.run_dynamics()

    @staticmethod
    def run_walk_test():
        nav_model = NavModel(NavAgent.TALOS_LEGS, Stimulus(StimuliTestWalk()))
        nav_model.add_walk_task()
        nav_model.run_dynamics()


if __name__ == '__main__':
    Experiment.run_walk_test()
