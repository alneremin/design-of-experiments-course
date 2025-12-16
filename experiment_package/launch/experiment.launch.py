from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    experiment = Node(
        package="experiment_package",
        executable="start_experiment",
        arguments=[],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(experiment)

    return ld