from ast import arg, arguments
from http.server import executable
from lib2to3.pytree import Node
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    db_arg = DeclareLaunchArgument("db",
                                   default_value="False",
                                   description="Database flag")
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda").robot_description(
            file_path="config/panda.urdf.xacro").robot_description_semantic(
                file_path="config/panda.srdf").trajectory_execution(
                    file_path="config/gripper_moveit_controllers.yaml").
        planning_pipelines(pipelines=["ompl", "chomp"]).to_moveit_configs())

    move_group_node = Node(package="moveit_ros_move_group",
                           executable="move_group",
                           output="screen",
                           parameters=[moveit_config.to_dict()],
                           arguments=["--ros-args", "--log-level", "info"])

    rviz_base = os.path.join(
        get_package_share_directory("panda_moveit_config"), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ],
    )

    static_tf_node = Node(package="tf2_ros",
                          executable="static_transform_publisher",
                          name="static_transform_publisher",
                          output="log",
                          arguments=[
                              "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
                              "world", "panda_link0"
                          ])
    robot_state_publisher = Node(package="robot_state_publisher",
                                 executable="robot_state_publisher",
                                 output="both",
                                 parameters=[moveit_config.robot_description])

    ros2_controllers_path = os.path.join(
        # get_package_share_dire
    )

    return LaunchDescription([db_arg, rviz_node])
