"""Closed-loop Stanley simulation on the waypoints of the loaded map.

    # terminal 1 - the map, launched the usual way (any mapping run)
    ros2 launch mola_slam_mapping localization.launch.py \\
        map_dir:=~/krowd_maps/20260913_103732 map_filename:=global_map_corrected.mm
    # terminal 2 - this file
    ros2 launch krowd_stanley_control stanley_dynamics_sim.launch.py

Stanley's commands drive an Ackermann bicycle model whose pose is fed back to
Stanley: what it commands is what the simulated vehicle does. Everything is on
/simulation/*; no CAN topic is used. The route is the waypoints.csv of the run
terminal 1 announced on /localization/map_dir (map_dir:=<dir> forces one), so
any map works, and the simulator publishes the vehicle's TF and footprint
itself, so terminal 1 needs no simulation-specific argument. Run it WITHOUT the
live sensor stack: its ESKF would publish base_link as well.

By default the simulation is as imperfect as the car (2026-09-23, measured on
stanley_tuning_0922_235531): steering dead time 0.24 s, drive MCU 1.7 km/h
under target, ESKF noise on the pose Stanley sees, a quantised /encoder that
Stanley's feedback watchdog and lead limit run on, and the ESKF's RTK-course
heading verification (a saved-prior heading 5 deg off until 3 straight fixes
above 1 m/s). ideal:=true switches all of it off.

Until 2026-09-20 this file started the localization itself, on one map.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from krowd_stanley_control.config_paths import source_config


def generate_launch_description():
    # The SOURCE yaml, exactly as path_tracking.launch.py reads it: the
    # simulation and the vehicle must never run different tuning because one
    # of them was rebuilt more recently than the other.
    vehicle_config = str(source_config("krowd_stanley_control", "vehicle.yaml"))
    tracking_config = str(source_config("krowd_stanley_control", "path_tracking.yaml"))
    sim_config = str(source_config("krowd_stanley_control", "sim_overrides.yaml"))
    heading_demo_config = str(source_config("krowd_stanley_control", "sim_heading_demo.yaml"))
    ideal_config = str(source_config("krowd_stanley_control", "sim_ideal.yaml"))
    cone_overrides = str(source_config("krowd_stanley_control", "sim_cone_overrides.yaml"))
    map_dir = LaunchConfiguration("map_dir")
    ideal = LaunchConfiguration("ideal")
    # stanley_node's overrides: ideal > heading demo > as-on-the-vehicle.
    stanley_sim_config = PythonExpression([
        "'", ideal_config, "' if '", ideal, "'.lower() in ('true', '1') else ('",
        heading_demo_config, "' if float('", LaunchConfiguration("heading_demo_sec"),
        "') > 0.0 else '", sim_config, "')",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_dir",
            default_value="",
            description=(
                "Empty (default) = the run the map terminal announced on "
                "/localization/map_dir. A directory forces that run's route."
            ),
        ),
        DeclareLaunchArgument(
            "speed_limit_kmph",
            default_value="0.0",
            description=(
                "Whole-route speed ceiling in km/h, as in the real-vehicle "
                "launch: 0 (default) = none, path_tracking.yaml's profile "
                "alone. Was 20.0 until 2026-09-25, which capped the simulation "
                "below the vehicle."
            ),
        ),
        DeclareLaunchArgument("initial_waypoint_index", default_value="0"),
        DeclareLaunchArgument("initial_lateral_offset_m", default_value="0.0"),
        DeclareLaunchArgument("initial_heading_offset_deg", default_value="0.0"),
        DeclareLaunchArgument(
            "steering_lag_sec",
            default_value="0.24",
            description="Steering dead time (measured 2026-09-22). 0 = ideal.",
        ),
        DeclareLaunchArgument(
            "speed_deficit_kmph",
            default_value="1.7",
            description="How far the drive MCU runs under /target_speed (measured 2026-09-22). 0 = ideal.",
        ),
        DeclareLaunchArgument(
            "ideal",
            default_value="false",
            description=(
                "true = the ideal model: no steering lag, no speed deficit, no "
                "pose noise, exact heading from the start, and stanley_node "
                "without feedback watchdog, lead limit or heading gate "
                "(config/sim_ideal.yaml)."
            ),
        ),
        DeclareLaunchArgument(
            "heading_prior_error_deg",
            default_value="5.0",
            description=(
                "How far the saved-prior heading is off until the simulated ESKF "
                "verifies it on the RTK course (the position is off with it, via "
                "the 2.5 m antenna lever arm). 0 = the prior is right."
            ),
        ),
        DeclareLaunchArgument(
            "noise_seed",
            default_value="0",
            description="Seed of the simulated ESKF / encoder noise.",
        ),
        DeclareLaunchArgument(
            "avoidance_demo_sec",
            default_value="0.0",
            description=(
                "0 (default) = the REAL cone_avoidance_1005 stack runs in the avoidance "
                "zone, fed by scripts/fake_lidar_drums.py (drum rows as a point cloud, "
                "see `obstacles`). > 0 = a stand-in instead (fake_cone_mission.py: "
                "straight at 7 km/h, DONE after this many seconds)."
            ),
        ),
        DeclareLaunchArgument(
            "obstacles",
            default_value="10:+1.0,17:-1.0",
            description=(
                "Drum rows for fake_lidar_drums.py: s_offset:lateral pairs, metres from "
                "the avoidance zone start, lateral + = left of the route. Default: a "
                "row on the left 10 m in, a row on the right 17 m in; 3 drums each. "
                "cone_avoidance_1005 plans only once TWO blocking groups are confirmed "
                "(trigger_min_groups 2) within its 15 m detect range, so rows more than "
                "about 12 m apart are never avoided."
            ),
        ),
        DeclareLaunchArgument(
            "cone_rviz",
            default_value="false",
            description="Also open cone_avoidance_1005's own RViz while it runs.",
        ),
        DeclareLaunchArgument(
            "heading_demo_sec",
            default_value="0.0",
            description=(
                "> 0: the simulator plays an ESKF whose heading becomes verified "
                "after this many seconds, and stanley_node runs with the heading "
                "gate and the unverified-heading creep ON (config/sim_heading_demo.yaml), "
                "so the wait -> creep -> tracking sequence can be watched in RViz."
            ),
        ),
        DeclareLaunchArgument(
            "steering_command_gain",
            default_value="8.0",
            description=(
                "Simulated column-command/road-wheel ratio. The default matches "
                "the current hardware calibration; the +/-2.5 rad MCU limit "
                "allows 17.9 deg at the road wheel (about 4.17 m radius)."
            ),
        ),
        LogInfo(
            msg=(
                "Closed-loop Stanley dynamics simulation: controller commands "
                "drive an Ackermann bicycle model; no CAN topics are used."
            )
        ),
        Node(
            package="krowd_stanley_control",
            executable="route_publisher_node",
            name="route_publisher_node",
            output="screen",
            parameters=[
                vehicle_config,
                tracking_config,
                sim_config,
                {
                    "map_dir": map_dir,
                    # Keep the closed loop in the route's native global frame.
                    # localization can refine map<->utm while starting; moving
                    # the controller's route after the simulator has initialized
                    # would leave the vehicle in the old coordinate system.
                    "output_frame_override": "utm",
                    "speed_limit_override_kmph": ParameterValue(
                        LaunchConfiguration("speed_limit_kmph"), value_type=float
                    ),
                    "use_sim_time": False,
                },
            ],
        ),
        Node(
            package="krowd_stanley_control",
            executable="vehicle_state_node",
            name="vehicle_state_node",
            output="screen",
            parameters=[
                {
                    "primary_odom_topic": "/simulation/odometry",
                    "fallback_odom_topic": "",
                    "output_frame": "",
                    "vehicle_state_topic": "/localization/vehicle_state",
                    "min_speed_for_curvature_kmph": 1.8,
                    "report_period_sec": 10.0,
                    "use_sim_time": False,
                },
            ],
        ),
        Node(
            package="krowd_stanley_control",
            executable="stanley_node",
            name="stanley_node",
            output="screen",
            parameters=[
                vehicle_config,
                tracking_config,
                stanley_sim_config,
                {
                    "angle_topic": "/simulation/target_angle",
                    "speed_topic": "/simulation/target_speed",
                    "brake_topic": "/simulation/brake",
                    "steering_command_gain": ParameterValue(
                        LaunchConfiguration("steering_command_gain"),
                        value_type=float,
                    ),
                    "use_sim_time": False,
                },
            ],
            remappings=[
                ("/target_angle", "/simulation/target_angle"),
                ("/target_speed", "/simulation/target_speed"),
                ("/brake", "/simulation/brake"),
            ],
        ),
        Node(
            package="krowd_stanley_control",
            executable="avoidance_supervisor_node",
            name="avoidance_supervisor_node",
            output="screen",
            parameters=[
                tracking_config,
                {
                    "command_topic": "/simulation/target_speed",
                    "launch_command": PythonExpression([
                        # The stand-in reads the straight-ahead command (steering zero)
                        # from vehicle.yaml itself.
                        "'ros2 run krowd_stanley_control fake_cone_mission.py --done-after ",
                        LaunchConfiguration("avoidance_demo_sec"),
                        " --angle-topic /simulation/target_angle --speed-topic "
                        "/simulation/target_speed --brake-topic /simulation/brake' if float('",
                        LaunchConfiguration("avoidance_demo_sec"),
                        "') > 0.0 else 'ros2 launch cone_avoidance_1005 "
                        "cone_avoidance_1005.launch.py use_can:=false use_rviz:=",
                        LaunchConfiguration("cone_rviz"),
                        " overrides:=", cone_overrides,
                        " output_angle_topic:=/simulation/target_angle "
                        "output_speed_topic:=/simulation/target_speed "
                        "output_brake_topic:=/simulation/brake'",
                    ]),
                    "use_sim_time": False,
                },
            ],
        ),
        Node(
            package="krowd_stanley_control",
            executable="fake_lidar_drums.py",
            name="fake_lidar_drums",
            output="screen",
            condition=IfCondition(PythonExpression(
                ["float('", LaunchConfiguration("avoidance_demo_sec"), "') <= 0.0"]
            )),
            arguments=["--obstacles", LaunchConfiguration("obstacles")]
            + (["--map", map_dir]),
        ),
        Node(
            package="krowd_stanley_control",
            executable="vehicle_dynamics_sim_node",
            name="vehicle_dynamics_sim_node",
            output="screen",
            parameters=[
                vehicle_config,
                {
                    "frame_id": "utm",
                    "ideal": ParameterValue(ideal, value_type=bool),
                    "heading_prior_error_deg": ParameterValue(
                        LaunchConfiguration("heading_prior_error_deg"), value_type=float
                    ),
                    "noise_seed": ParameterValue(
                        LaunchConfiguration("noise_seed"), value_type=int
                    ),
                    "encoder_topic": "/simulation/encoder",
                    # The RViz "Driven Path" display. map_odom_tf_node, its
                    # publisher on the vehicle, needs a MOLA LiDAR pose and
                    # stays silent here, so the two never both publish.
                    "driven_path_topic": "/driven_path",
                    "steering_lag_sec": ParameterValue(
                        LaunchConfiguration("steering_lag_sec"), value_type=float
                    ),
                    "speed_deficit_kmph": ParameterValue(
                        LaunchConfiguration("speed_deficit_kmph"), value_type=float
                    ),
                    "heading_verified_after_sec": ParameterValue(
                        LaunchConfiguration("heading_demo_sec"), value_type=float
                    ),
                    # The vehicle's own RViz config draws /vehicle_marker (the
                    # live pose cuboid from map_odom_tf_node, silent here: no
                    # GNSS pose exists in a simulation) and has no display for
                    # /simulation/vehicle_marker, so the footprint goes there.
                    "vehicle_marker_topic": "/vehicle_marker",
                    "steering_command_gain": ParameterValue(
                        LaunchConfiguration("steering_command_gain"),
                        value_type=float,
                    ),
                    "initial_waypoint_index": ParameterValue(
                        LaunchConfiguration("initial_waypoint_index"),
                        value_type=int,
                    ),
                    "initial_lateral_offset_m": ParameterValue(
                        LaunchConfiguration("initial_lateral_offset_m"),
                        value_type=float,
                    ),
                    "initial_heading_offset_deg": ParameterValue(
                        LaunchConfiguration("initial_heading_offset_deg"),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
