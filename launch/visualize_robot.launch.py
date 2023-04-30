import os

from ament_index_python import get_package_prefix

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart



def launch_setup(context, *args, **kwargs):

    # need to be added to allow stl to be identified from gazebo side
    pkg_share_path = os.pathsep + \
        os.path.join(get_package_prefix('haruto_description'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # rviz config file path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("haruto_description"), "rviz",
         "visualize_robot.rviz"]
    )

    # xacro command for model generation
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package),
                 "urdf", description_file]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # robot state publisher for publishing robot model
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description]
    )

    # joint state publisher gui
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both"
    )

    # rviz node for visualizing robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    delay_joint_state_publisher_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[joint_state_publisher_gui],
        )
    )

    delay_rviz_after_joint_state_publisher_gui = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_publisher_gui,
            on_start=[rviz_node],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        delay_joint_state_publisher_after_robot_state_publisher,
        delay_rviz_after_joint_state_publisher_gui,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="haruto_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="omni_robot.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
