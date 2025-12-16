from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
import numpy as np
from experiment_package.utils import init_csv, save_to_csv
import time
from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
from rcl_interfaces.msg import ParameterType

class Experiment:

    def __init__(self, path_to_exp_plan, gz_control, nav_control, control_factors, output_path, columns_names):
        self.exp_plan = np.load(path_to_exp_plan, allow_pickle=True)
        self.gz_control: GazeboControl = gz_control
        self.nav_control: NavigationControl = nav_control
        self.control_factors = control_factors
        self.output_path = output_path
        self.columns_names = columns_names
        self.current_trial = None

    def create_map(self):
        self.gz_control.spawn_obstacle_by_url("obstacle1", self.control_factors["obstacle_path"], self.control_factors["obstacle1_pose"])
        self.gz_control.spawn_obstacle_by_url("obstacle2", self.control_factors["obstacle_path"], self.control_factors["obstacle2_pose"])

    def reset_environment(self, input_factors):
        self.nav_control.reset_navigation_stack()
        self.gz_control.move_robot(self.control_factors["robot_name"], self.control_factors["robot_start_gz_pose"])
        self.nav_control.change_nav2_parameter("FollowPath.PathDist.scale", ParameterType.PARAMETER_DOUBLE, input_factors["path_dist"])
        self.nav_control.change_nav2_parameter("FollowPath.GoalDist.scale", ParameterType.PARAMETER_DOUBLE, input_factors["goal_dist"])
        self.nav_control.startup_navigation_stack()
        time.sleep(5.0)
        self.nav_control.update_robot_pose(self.control_factors["amcl_start_robot_pose"])
        time.sleep(5.0)

    def do_trial(self, trial):
        path_dist = float(trial[0])
        goal_dist = float(trial[1])
        index = int(trial[2])

        input_factors = {
            "path_dist": path_dist,
            "goal_dist": goal_dist
        }

        self.current_trial = trial
        self.nav_control.node.get_logger().info(f"Experiment #{index}: In progress..")
        self.nav_control.node.get_logger().info(f"Experiment #{index}: Reset environment..")
        self.reset_environment(input_factors)
        self.nav_control.node.get_logger().info(f"Experiment #{index}: Start navigation..")
        nav_status, nav_time = self.nav_control.start_robot_navigation(self.control_factors["amcl_target_robot_pose"])
        self.nav_control.node.get_logger().info(f"Experiment #{index}: Navigation finished with status: {nav_status}, navigation time: {nav_time}")
        response = {
            "nav_status": nav_status,
            "nav_time": nav_time
        }
        return response


    def save_results(self, data, columns, output_path):
        save_to_csv(data, columns, output_path)


    def do_experiments(self):
        init_csv(self.columns_names, self.output_path)
        self.create_map()

        for trial in self.exp_plan:
            index = int(trial[2])
            response = self.do_trial(trial)

            data = { 
                "index": index,
                "path_dist": float(trial[0]),
                "goal_dist": float(trial[1])
            }

            data.update(response)
            self.nav_control.node.get_logger().info(f"Experiment #{index}: Save results..")
            self.save_results([data], self.columns_names, self.output_path)
            self.nav_control.node.get_logger().info(f"Experiment #{index}: Results saved.")
