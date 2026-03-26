"""
simulation.launch.py
--------------------
Gazebo szimulációs környezet + összes ROS2 node egyszerre indítása.

Indítás: ros2 launch visual_servoing simulation.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_vs   = get_package_share_directory("visual_servoing")
    pkg_desc = get_package_share_directory("ur_description")

    params_file = os.path.join(pkg_vs, "config", "params.yaml")
    world_file  = os.path.join(pkg_vs, "worlds", "assembly_world.world")
    urdf_file   = os.path.join(pkg_desc, "urdf", "ur3e.urdf.xacro")

    return LaunchDescription([

        # ── 1. Gazebo szimulátor ────────────────────────────────────────
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen"
        ),

        # ── 2. Robot model publikálása ──────────────────────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_file).read()}],
            output="screen"
        ),

        # ── 3. ROS2-Gazebo híd (topic-ok összekapcsolása) ───────────────
        # Kis késleltetés hogy a Gazebo elinduljon
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                     "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                     "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                     "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen"
            ),
        ]),

        # ── 4. Vezérlők aktiválása ──────────────────────────────────────
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller",
                     "--set-state", "active",
                     "joint_state_broadcaster"],
                output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller",
                     "--set-state", "active",
                     "joint_trajectory_controller"],
                output="screen"
            ),
        ]),

        # ── 5. Alkalmazás node-ok ───────────────────────────────────────
        TimerAction(period=6.0, actions=[

            Node(
                package="visual_servoing",
                executable="vision_node",
                name="vision_node",
                parameters=[params_file],
                output="screen"
            ),

            Node(
                package="visual_servoing",
                executable="controller_node",
                name="controller_node",
                parameters=[params_file],
                output="screen"
            ),
        ]),

        # ── 6. RViz2 vizualizáció ───────────────────────────────────────
        TimerAction(period=4.0, actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(pkg_vs, "config", "visual_servoing.rviz")],
                output="screen"
            ),
        ]),
    ])
