from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    delta_x_arg = DeclareLaunchArgument("delta_x", default_value="0.0")
    delta_y_arg = DeclareLaunchArgument("delta_y", default_value="0.0")
    delta_yaw_arg = DeclareLaunchArgument("delta_yaw", default_value="0.0")
    transform_tolerance_arg = DeclareLaunchArgument(
        "transform_tolerance", default_value="0.1"
    )
    odom_frame_id_arg = DeclareLaunchArgument("odom_frame_id", default_value="odom")
    base_frame_id_arg = DeclareLaunchArgument(
        "base_frame_id", default_value="base_link"
    )
    global_frame_id_arg = DeclareLaunchArgument("global_frame_id", default_value="map")

    fake_localization_node = Node(
        package="fake_localization",
        executable="fake_localization",
        output="screen",
        parameters=[
            {"delta_x": LaunchConfiguration("delta_x")},
            {"delta_y": LaunchConfiguration("delta_y")},
            {"delta_yaw": LaunchConfiguration("delta_yaw")},
            {"transform_tolerance": LaunchConfiguration("transform_tolerance")},
            {"odom_frame_id": LaunchConfiguration("odom_frame_id")},
            {"base_frame_id", LaunchConfiguration("base_frame_id")},
            {"global_frame_id": LaunchConfiguration("global_frame_id")},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(delta_x_arg)
    ld.add_action(delta_y_arg)
    ld.add_action(delta_yaw_arg)
    ld.add_action(transform_tolerance_arg)
    ld.add_action(odom_frame_id_arg)
    ld.add_action(base_frame_id_arg)
    ld.add_action(global_frame_id_arg)
    ld.add_action(fake_localization_node)
    return ld
