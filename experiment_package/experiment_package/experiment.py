import time
import numpy as np

from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
from rcl_interfaces.msg import ParameterType

from experiment_package.utils import init_csv, save_to_csv

class Experiment:

    def __init__(self, path_to_exp_plan, gz_control, nav_control, control_factors, output_path, columns_names, node):
        self.exp_plan = np.load(path_to_exp_plan, allow_pickle=True)
        self.gz_control: GazeboControl = gz_control
        self.nav_control: NavigationControl = nav_control
        self.control_factors = control_factors
        self.output_path = output_path
        self.columns_names = columns_names
        self.current_trial = None
        self.node = node


    def reset_trial(self, input_factors):
        self.nav_control.reset_navigation_stack()
        time.sleep(0.5)
        self.gz_control.move_robot(self.control_factors["robot_name"], self.control_factors["robot_start_gz_pose"])
        self.nav_control.change_nav2_parameter("FollowPath.vx_samples", ParameterType.PARAMETER_INTEGER, input_factors["vx_samples"])
        time.sleep(0.5)
        self.nav_control.change_nav2_parameter("FollowPath.vy_samples", ParameterType.PARAMETER_INTEGER, input_factors["vy_samples"])
        time.sleep(0.5)
        self.nav_control.change_nav2_parameter("FollowPath.vtheta_samples", ParameterType.PARAMETER_INTEGER, input_factors["vtheta_samples"])
        time.sleep(0.5)
        self.nav_control.startup_navigation_stack()
        time.sleep(3)
        self.nav_control.update_robot_pose(self.control_factors["amcl_start_robot_pose"])
        time.sleep(3)

    def do_trial(self, trial):

        vx_samples = trial[0]
        vy_samples = trial[1]
        vtheta_samples = trial[2]
        index = trial[3]

        input_factors = {
            "vx_samples": int(vx_samples),
            "vy_samples": int(vy_samples),
            "vtheta_samples": int(vtheta_samples)
        }

        self.current_trial = trial

        self.node.get_logger().info(f"Experiment #{index}: In progress..")

        self.node.get_logger().info(f"Experiment #{index}: Reset environment..")
        self.reset_trial(input_factors)

        self.node.get_logger().info(f"Experiment #{index}: Start navigation..")
        cpu, ram = self.nav_control.start_robot_navigation(self.control_factors["amcl_target_robot_pose"])

        self.node.get_logger().info(f"Experiment #{index}: avg_cpu: {sum(cpu)/len(cpu)}, max_cpu: {max(cpu)}, avg_ram: {sum(ram)/len(ram)}, max_ram: {max(ram)}")

        response = {
            "avg_cpu": round(sum(cpu)/len(cpu), 3),
            "max_cpu": round(max(cpu), 3),
            "avg_ram": round(sum(ram)/len(ram), 3),
            "max_ram": round(max(ram), 3)
        }

        return response



    def save_results(self, data, columns, output_path):
        save_to_csv(data, columns, output_path)


    def do_experiments(self):
        init_csv(self.columns_names, self.output_path)

        for trial in self.exp_plan:
            
            index = trial[3]
            response = self.do_trial(trial)

            data = { 
                "index": trial[3],
                "vx_samples": trial[0],
                "vy_samples": trial[1],
                "vtheta_samples": trial[2]
            }
            data.update(response)

            self.node.get_logger().info(f"Experiment #{index}: Save results..")
            self.save_results([data], self.columns_names, self.output_path)
            self.node.get_logger().info(f"Experiment #{index}: Results saved.")
