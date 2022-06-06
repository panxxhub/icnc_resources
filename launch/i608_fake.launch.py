import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def get_absolute_file_path(package_name, file_path):
    package_path = get_package_share_directory(package_name=package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def load_yaml(package_name, file_path):
    absolute_file_path = get_absolute_file_path(package_name, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_configs = (
        MoveItConfigsBuilder("icnc_resources_i608")
        .robot_description(file_path="config/i608.urdf.xacro")
        .robot_description_semantic(file_path="config/i608_arm.srdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/i608_moveit_controllers.yaml", moveit_manage_controllers=False)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    ''' dump to yaml file '''
    moveit_config_dict = moveit_configs.to_dict()
    joint_limits = load_yaml(
        "icnc_resources_i608_moveit_config", "config/joint_limits.yaml")
    moveit_config_dict["robot_description_planning"] += joint_limits
    with open("moveit_config.yaml", "w") as f:
        yaml.dump(moveit_config_dict, f, default_flow_style=False)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_configs.to_dict()]
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_configs.robot_description],
    )

    return LaunchDescription([
        static_tf_node,
        robot_state_publisher,
        move_group_node,
    ])
