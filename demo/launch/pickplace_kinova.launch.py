import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("/kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="/root/kortex2_ws/src/ros2_kortex/kortex_description/robots/gen3_robotiq_2f_85.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    pick_place_demo = Node(
        package="moveit_task_constructor_demo",
        executable="pick_place_demo",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("moveit_task_constructor_demo"),
                "config",
                "kinova_config.yaml",
            ),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([pick_place_demo])