import threading
import rclpy
import sys
from pathlib import Path
from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
from rcl_interfaces.msg import ParameterType
from experiment_package.experiment import Experiment

def main(argv=sys.argv):

    rclpy.init()
    node = rclpy.create_node('experiment')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    gz_control = GazeboControl(node)
    nav_control = NavigationControl(node)
    current_file = Path(__file__).resolve()
    experiment_package_path = current_file / "../../../../../../../src/experiment_package"
    obstacle_path = (experiment_package_path / "worlds" / "obstacle.sdf").resolve()
    input_path = (experiment_package_path / "input.npy").resolve()
    output_path = (experiment_package_path / "output.csv").resolve()

    control_factors = {
        "robot_start_gz_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_start_robot_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_target_robot_pose": [7.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "obstacle1_pose": [2.0, 0.925, 0.0, 0.0, 0.0, 0.0, 1.0],
        "obstacle2_pose": [6.0, 6.385, 0.0, 0.0, 0.0, 0.0, 1.0],
        "obstacle3_pose": [4.5, 4.45, 0.0, 0.0, 0.0, 0.75, 0.75],
        "obstacle4_pose": [1.5, 4.0, 0.0, 0.0, 0.0, 1.0, 1.0],
        "obstacle_path": str(obstacle_path),
        "robot_name": "artbul"
    }

    columns_names = ["index", "path_align", "goal_align", "nav_status", "nav_time"]
    experiment = Experiment(str(input_path), gz_control, nav_control, control_factors, str(output_path), columns_names)
    node.get_logger().info("Experiment started.")
    experiment.do_experiments()

    rclpy.spin(node)
    executor_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(argv=sys.argv)
