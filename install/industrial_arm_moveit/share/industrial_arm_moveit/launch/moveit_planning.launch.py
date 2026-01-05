import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                'share/industrial_arm_moveit/config/robot_description.urdf.xacro')
        ]),
        value_type=str
    )

    robot_description_semantic = ParameterValue(
        open(os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],
            'share/industrial_arm_moveit/config/robot_description_semantic.srdf')).read(),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share/industrial_arm_moveit/config/kinematics.yaml'),
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share/industrial_arm_moveit/config/ompl_planning.yaml'),
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share/industrial_arm_moveit/config/joint_limits.yaml'),
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share/industrial_arm_moveit/config/moveit_controllers.yaml'),
            ]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
        )

    ])
