from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
import numpy as np
import time
from rcl_interfaces.msg import ParameterType
from experiment_package.utils import init_csv, save_to_csv


class Experiment:

    def __init__(self, path_to_exp_plan, gz_control, nav_control, control_factors, output_path, columns_names):
        self.exp_plan = np.load(path_to_exp_plan, allow_pickle=True)
        self.gz_control: GazeboControl = gz_control
        self.nav_control: NavigationControl = nav_control
        self.control_factors = control_factors
        self.output_path = output_path
        self.columns_names = columns_names
        self.current_trial = None

    def reset_environment(self, input_factors):
        self.nav_control.reset_navigation_stack()
        
        self.gz_control.move_robot(self.control_factors["robot_name"], self.control_factors["robot_start_gz_pose"])
        
        old_obstacle_name, new_obstacle_name, new_obstacle_url = input_factors["obstacle_info"]
        
        if old_obstacle_name:
            self.gz_control.remove_obstacle(old_obstacle_name)
        self.gz_control.spawn_obstacle_by_url(new_obstacle_name, new_obstacle_url, self.control_factors["obstacle_pose"])

        vx, vy, vth = input_factors["velocities"]
        self.nav_control.change_nav2_parameter("FollowPath.max_vel_x", ParameterType.PARAMETER_DOUBLE, vx)
        self.nav_control.change_nav2_parameter("FollowPath.max_vel_y", ParameterType.PARAMETER_DOUBLE, vy)
        self.nav_control.change_nav2_parameter("FollowPath.max_vel_theta", ParameterType.PARAMETER_DOUBLE, vth)
        
        self.nav_control.startup_navigation_stack()
        time.sleep(2.0)
        self.nav_control.update_robot_pose(self.control_factors["amcl_start_robot_pose"])
        time.sleep(2.0)


    def do_trial(self, trial):
        
        obstacle_path = trial[0]
        max_vel_x = float(trial[1])
        max_vel_y = float(trial[2])
        max_vel_theta = float(trial[3])
        index = int(trial[4])

        current_obstacle_name = f"obstacle_{index}"
        prev_obstacle_name = "" 
        if self.current_trial is not None:
             prev_obstacle_name = f"obstacle_{int(self.current_trial[4])}"

        input_factors = {
            "obstacle_info": (prev_obstacle_name, current_obstacle_name, obstacle_path),
            "velocities": (max_vel_x, max_vel_y, max_vel_theta)
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

        for trial in self.exp_plan:
            
            index = int(trial[4])
            response = self.do_trial(trial)
            
            data = { 
                "index": index,
                "max_vel_x": float(trial[1]),
                "max_vel_y": float(trial[2]),
                "max_vel_theta": float(trial[3])
            }
            data.update(response)

            self.nav_control.node.get_logger().info(f"Experiment #{index}: Save results..")
            self.save_results([data], self.columns_names, self.output_path)
            self.nav_control.node.get_logger().info(f"Experiment #{index}: Results saved.")
