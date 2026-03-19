from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # OpenVR DLL folder (so overlay.exe can find openvr_api.dll at runtime)
    openvr_bin = os.path.join(
        os.path.dirname(__file__),  # .../src/VR/launch
        "..", "thirdparty", "openvr", "bin", "win64"
    )
    openvr_bin = os.path.normpath(openvr_bin)

    # Prepend openvr bin to PATH
    new_path = openvr_bin + os.pathsep + os.environ.get("PATH", "")

    rviz_config_file = os.path.join(
        get_package_share_directory("VR"),
        "launch", 
        "rviz.rviz"
    )

    overlay_node = Node(
        package="VR",
        executable="overlay",
        name="overlay",
        output="screen",
        emulate_tty=True,
    )

    tracker_node = Node(
        package="VR",
        executable="tracker",
        name="tracker",
        output="screen",
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable(name="PATH", value=new_path),
        overlay_node,
        tracker_node,
        rviz_node,
    ])
