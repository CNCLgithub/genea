from mlr.share.projects.navigation.utils.navigation_utils import NavTask, NavAgent


class TurnTask(NavTask):
    def __init__(self, rotate_to_angle_rad):
        super().__init__(NavTask.TURN)

        self._rotate_to_angle_rad = rotate_to_angle_rad

    def get_task_problem(self, nav_agent: NavAgent):
        return None

    def run_task(self, nav_agent):
        next_agent_pose = NavAgent.rotate_to(nav_agent.get_current_pose(), self._rotate_to_angle_rad)
        nav_agent.set_ref_pose(next_agent_pose)
        nav_agent.update_current_pose(next_agent_pose)
