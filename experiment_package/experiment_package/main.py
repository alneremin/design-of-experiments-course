import threading
import rclpy
import sys

from rcl_interfaces.msg import ParameterType

from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
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

    control_factors = {
        "robot_start_gz_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_start_robot_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_target_robot_pose": [3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "robot_name": "artbul"
    }

    columns_names = ["index", "vx_samples", "vy_samples", "vtheta_samples", "avg_cpu" , "max_cpu", "avg_ram", "max_ram"]
    nav_control.update_robot_pose([1,1,0,0,0,0,1])
    experiment = Experiment("/home/timur/controllers/src/experiment_package/input.npy", gz_control, nav_control, control_factors, "/home/timur/controllers/src/experiment_package/output.csv", columns_names, node)
    experiment.do_experiments()

if __name__ == "__main__":
    main(argv=sys.argv)