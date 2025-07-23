
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    world_name = LaunchConfiguration("world_name")

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="empty_table"
    )
    # Set software rendering
    set_software_rendering = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE", value="1"
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_description"),
            "launch",
            "gazebo.launch.py"
        )),
        launch_arguments={
            "world_name": world_name
        }.items()
    )

    controller_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bot_controller"),
            "launch",
            "controller.launch.py"
        ))
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bot_description"),
            "rviz",
            "display.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    edge_avoidance_node = Node(
        package="bot_script",
        executable="edge_detection",
        name="edge_avoidance_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        world_name_arg,
        set_software_rendering, 
        gazebo,
        controller_only,
        rviz,
        edge_avoidance_node,
    ])
