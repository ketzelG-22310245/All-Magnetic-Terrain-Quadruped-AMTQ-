import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("quadruped_description")
    pkg_prefix = get_package_prefix("quadruped_description")

    install_dir = os.path.join(pkg_prefix, "share")
    urdf_path = os.path.join(pkg_share, "quadruped", "robot.urdf")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    robot_desc = robot_desc.replace("$(find quadruped_description)", pkg_share)

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[install_dir, ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", robot_desc, "-name", "quadruped", "-z", "0.3"],
        output="screen",
    )

    # 1. Cargar el broadcaster primero
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 2. Cargar el leg_position_controller DESPUÉS de que termine el broadcaster
    leg_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_position_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            set_gz_resource_path,
            robot_state_publisher,
            gazebo,
            spawn_robot,
            joint_state_broadcaster,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[leg_controller],
                )
            ),
        ]
    )
