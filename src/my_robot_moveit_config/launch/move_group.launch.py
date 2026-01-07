from launch import LaunchDescription
from launch_ros.actions import SetParameter

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot",
            package_name="my_robot_moveit_config"
        )
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # Force simulation time for ALL ROS nodes started after this
    ld.add_action(SetParameter(name="use_sim_time", value=True))

    # MoveIt (move_group + RViz + robot_state_publisher)
    ld.add_action(generate_move_group_launch(moveit_config))

    return ld

