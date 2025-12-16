from rcl_interfaces.msg import ParameterType
import threading
import rclpy
import sys
from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
import time
from experiment_package.experiment import Experiment

def main(argv=sys.argv):

    rclpy.init()
    node = rclpy.create_node('experiment')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    nav_control = NavigationControl(node)
    gz_control = GazeboControl()

    path_to_matrix = "/home/nail/experiment_ws/src/experiment_package/input.npy"
    control_factors = {
        "robot_start_gz_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_start_robot_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_target_robot_pose": [7.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "obstacle_pose": [4.0, 4.0, 0.5, 0.0, 0.0, 0.3824995, 0.9239557],
        "robot_name": "artbul"
    }

    columns_names = ["index", "max_vel_x", "max_vel_y", "max_vel_theta", "nav_status", "nav_time"]
    
    experiment = Experiment(path_to_matrix, gz_control, nav_control, control_factors, "/home/nail/experiment_ws/src/experiment_package/output.csv", columns_names)
    experiment.do_experiments()

    rclpy.spin(node)
    executor_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(argv=sys.argv)