"""Terminal 2 of a Stanley drive: follow the waypoints of the loaded map.

    # terminal 1 - the map (any mapping run), left running
    ros2 launch mola_slam_mapping localization.launch.py \\
        map_dir:=~/krowd_maps/20260913_103732 map_filename:=global_map_corrected.mm
    # terminal 2 - this file
    ros2 launch krowd_stanley_control stanley_waypoint_follow.launch.py                      # check
    ros2 launch krowd_stanley_control stanley_waypoint_follow.launch.py drive_enabled:=true  # drive

Route publisher, vehicle state and (drive_enabled:=true) stanley_node. No map
argument: the route is the waypoints.csv of whichever run terminal 1 announced
on /localization/map_dir, so the two cannot end up on different maps. Until
2026-09-20 this file started the localization itself, with one map as its
default; map_dir:=<dir> still forces a route directory by hand.

With drive_enabled:=true Stanley publishes /target_angle, /target_speed and
/brake for ros_can_bridge. Without it nothing is commanded.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tracking_share = Path(get_package_share_directory("krowd_stanley_control"))

    drive_enabled = LaunchConfiguration("drive_enabled")
    angle_topic = LaunchConfiguration("angle_topic")
    speed_topic = LaunchConfiguration("speed_topic")
    brake_topic = LaunchConfiguration("brake_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_dir",
            default_value="",
            description=(
                "Empty (default) = follow the run the map terminal "
                "(mola_slam_mapping localization.launch.py) announced on "
                "/localization/map_dir. A directory forces that "
                "run's waypoints.csv / corridor.csv instead."
            ),
        ),
        DeclareLaunchArgument(
            "drive_enabled",
            default_value="true",
            description=(
                "true starts Stanley actuator output on /target_angle, "
                "/target_speed and /brake; false publishes the route and the "
                "vehicle state only"
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "primary_odom_topic", default_value="/ekf/odometry"
        ),
        DeclareLaunchArgument(
            "fallback_odom_topic", default_value="/localization/reference_pose"
        ),
        DeclareLaunchArgument("vehicle_state_frame", default_value="map"),
        DeclareLaunchArgument("angle_topic", default_value="/target_angle"),
        DeclareLaunchArgument("speed_topic", default_value="/target_speed"),
        DeclareLaunchArgument("brake_topic", default_value="/brake"),
        DeclareLaunchArgument(
            "steering_feedback_topic",
            default_value="/encoder",
            description=(
                "Measured steering angle. stanley_node brakes without it on the "
                "vehicle (steering_feedback_required in path_tracking.yaml)."
            ),
        ),
        DeclareLaunchArgument(
            "goal_mode",
            default_value="STOP_AT_END",
            description=(
                "STOP_AT_END ramps the target speed to zero at the last "
                "waypoint (max_decel_mps2). FOLLOW_UNTIL_REPLACED (the YAML "
                "default) has no ramp and brakes at goal_tolerance_m = 1 m "
                "from the end, but the measured E-stop distance is 1.5 m at "
                "5 km/h, so the vehicle would stop past the route."
            ),
        ),
        DeclareLaunchArgument(
            "speed_limit_kmph",
            default_value="0.0",
            description=(
                "Whole-route speed ceiling in km/h; it only ever LOWERS "
                "path_tracking.yaml's speeds (20 on straights, 10 in bends). "
                "0 (default) = no ceiling. Lower it for a cautious run, e.g. "
                "speed_limit_kmph:=5.0."
            ),
        ),
        LogInfo(
            condition=IfCondition(drive_enabled),
            msg=[
                "STANLEY DRIVE ENABLED: publishing ",
                angle_topic,
                ", ",
                speed_topic,
                " and ",
                brake_topic,
                ". Ensure Stanley is the only actuator topic publisher.",
            ],
        ),
        LogInfo(
            condition=UnlessCondition(drive_enabled),
            msg=(
                "Stanley drive disabled: route and vehicle state only, no "
                "actuator commands."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(tracking_share / "launch" / "path_tracking.launch.py")
            ),
            launch_arguments={
                "map_dir": LaunchConfiguration("map_dir"),
                "controller": drive_enabled,
                "controller_type": "stanley",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "primary_odom_topic": LaunchConfiguration("primary_odom_topic"),
                "fallback_odom_topic": LaunchConfiguration(
                    "fallback_odom_topic"
                ),
                "vehicle_state_frame": LaunchConfiguration(
                    "vehicle_state_frame"
                ),
                "angle_topic": angle_topic,
                "speed_topic": speed_topic,
                "brake_topic": brake_topic,
                "steering_feedback_topic": LaunchConfiguration(
                    "steering_feedback_topic"
                ),
                "speed_limit_kmph": LaunchConfiguration("speed_limit_kmph"),
                "goal_mode": LaunchConfiguration("goal_mode"),
            }.items(),
        ),
    ])
