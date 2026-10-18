"""Bring up route publication, the vehicle-state adapter and an optional controller.

Controllers are disabled by default so mission-specific controllers can own the
actuator topics safely. Set controller:=true to start stanley_node, the only
controller in this package (Pure Pursuit is krowd_slam/krowd_path_tracking's).

Runs alongside a localization stack that is already producing a pose - it does
not start one. On the vehicle that is mola_slam_mapping's localization.launch.py.

Every path is a launch argument and every default that names a location is
relative to $HOME, so moving this package to another machine needs no edit
inside it.
"""

import tempfile

import yaml
from krowd_stanley_control.config_paths import source_config
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _controller_selected(launch_controller, controller_type, name):
    """Start `name` only when controllers are enabled and it is the one selected."""

    return IfCondition(
        PythonExpression(
            [
                "'",
                launch_controller,
                "'.lower() in ('true', '1') and '",
                controller_type,
                "' == '",
                name,
                "'",
            ]
        )
    )


class NodeParamsFile(Substitution):
    """Launch-argument overrides written as a params file keyed by node name.

    A plain dict in Node(parameters=[...]) is written under `/**:`, and rcl
    lets a node-name key beat `/**` whatever the file order - so every key
    path_tracking.yaml sets under `route_publisher_node:` etc. silently ignored
    its launch argument (goal_mode, the odom topics, ...). Keyed by the node
    name and listed after the YAML files, these overrides win as intended.

    Keys in `optional` are left out when they resolve to an empty string, so
    the YAML value stands unless the launch argument is actually set.
    """

    def __init__(self, node_name, overrides, optional=()):
        super().__init__()
        self._node_name = node_name
        self._overrides = overrides
        self._optional = set(optional)

    def _resolve(self, context, value):
        if isinstance(value, ParameterValue):
            text = perform_substitutions(
                context, normalize_to_list_of_substitutions(value.value)
            )
            return value.value_type(text) if value.value_type else text
        if isinstance(value, (Substitution, str)) or isinstance(value, list):
            text = perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
            # Same typing as launch_ros gives a dict value: YAML rules, except
            # that an empty string stays a string.
            return yaml.safe_load(text) if text.strip() else text
        return value

    def perform(self, context):
        values = {}
        for key, value in self._overrides.items():
            resolved = self._resolve(context, value)
            if key in self._optional and resolved in ("", None):
                continue
            values[key] = resolved
        handle = tempfile.NamedTemporaryFile(
            "w", prefix=f"{self._node_name}_overrides_", suffix=".yaml",
            delete=False,
        )
        with handle:
            yaml.safe_dump({self._node_name: {"ros__parameters": values}}, handle)
        return handle.name


def generate_launch_description():
    # Source-tree YAML (see config_paths.source_config): edit config/*.yaml, relaunch.
    vehicle_params = str(source_config("krowd_stanley_control", "vehicle.yaml"))
    cone_overrides = str(source_config("krowd_stanley_control", "cone_standalone_overrides.yaml"))
    tracking_params = str(source_config("krowd_stanley_control", "path_tracking.yaml"))

    map_dir = LaunchConfiguration("map_dir")
    waypoints_csv = LaunchConfiguration("waypoints_csv")
    corridor_csv = LaunchConfiguration("corridor_csv")
    vehicle_config = LaunchConfiguration("vehicle_config")
    tracking_config = LaunchConfiguration("tracking_config")
    launch_controller = LaunchConfiguration("controller")
    controller_type = LaunchConfiguration("controller_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "map_dir",
            default_value="",
            description=(
                "Mapping run directory holding waypoints.csv and corridor.csv. "
                "Empty (the default) follows whichever run localization loaded, "
                "which it announces on /localization/map_dir - so the map is "
                "named once, in localization.launch.py, and the two cannot end "
                "up on different runs."
            ),
        ),
        DeclareLaunchArgument(
            "waypoints_csv",
            default_value="",
            description="Overrides <map_dir>/waypoints.csv when set.",
        ),
        DeclareLaunchArgument(
            "corridor_csv",
            default_value="",
            description="Overrides <map_dir>/corridor.csv when set.",
        ),
        DeclareLaunchArgument(
            "vehicle_config",
            default_value=vehicle_params,
            description="Vehicle geometry YAML.",
        ),
        DeclareLaunchArgument(
            "tracking_config",
            default_value=tracking_params,
            description="Path tracking tuning YAML.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=(
                "true when replaying a bag with --clock. These nodes compare "
                "timestamps they produce against timestamps they consume - the "
                "trajectory's valid_until, the vehicle state's age - so running "
                "them on a different clock from the localization stack makes "
                "everything look either fresh forever or permanently expired."
            ),
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="false",
            description=(
                "Start the controller chosen by controller_type. Defaults to false "
                "so mission controllers such as stop_mission can own /target_angle, "
                "/target_speed and /brake without competing publishers. Set true "
                "explicitly for normal route driving."
            ),
        ),
        DeclareLaunchArgument(
            "controller_type",
            default_value="stanley",
            choices=["stanley"],
            description=(
                "stanley_node - the only controller in this package since "
                "2026-09-20. Pure Pursuit lives in krowd_slam/krowd_path_tracking."
            ),
        ),
        DeclareLaunchArgument("angle_topic", default_value="/target_angle"),
        DeclareLaunchArgument("speed_topic", default_value="/target_speed"),
        DeclareLaunchArgument("brake_topic", default_value="/brake"),
        DeclareLaunchArgument(
            "steering_feedback_topic",
            default_value="/encoder",
            description="Measured steering angle (ros_can_bridge EncoderAngleStamped).",
        ),
        DeclareLaunchArgument(
            "primary_odom_topic", default_value="/ekf/odometry"
        ),
        DeclareLaunchArgument(
            "fallback_odom_topic", default_value="/localization/reference_pose"
        ),
        DeclareLaunchArgument("vehicle_state_frame", default_value="map"),
        DeclareLaunchArgument(
            "speed_limit_kmph",
            default_value="0.0",
            description="Optional ceiling for the complete route speed profile.",
        ),
        DeclareLaunchArgument(
            "cone_launch_command",
            default_value=(
                "ros2 launch cone_avoidance_1005 cone_avoidance_1005.launch.py "
                "use_can:=false use_rviz:=false overrides:=" + cone_overrides
            ),
            description=(
                "What avoidance_supervisor_node runs while stanley_node has handed "
                "the vehicle over in an avoidance zone (path_tracking.yaml "
                "avoidance_handoff_enabled). The overrides file routes the cone "
                "controller through its command_guard; without it cone_control "
                "publishes /target_angle and /target_speed itself (2026-10-08)."
            ),
        ),
        DeclareLaunchArgument(
            "goal_mode",
            default_value="",
            description=(
                "Overrides path_tracking.yaml's route_publisher goal_mode when "
                "set (STOP_AT_END / FOLLOW_UNTIL_REPLACED / ...). Empty keeps "
                "the YAML value."
            ),
        ),
    ]

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=[
                    "path tracking parameters: ",
                    vehicle_config,
                    " + ",
                    tracking_config,
                ]
            ),
            Node(
                package="krowd_stanley_control",
                executable="route_publisher_node",
                name="route_publisher_node",
                output="screen",
                parameters=[
                    vehicle_config,
                    tracking_config,
                    NodeParamsFile(
                        "route_publisher_node",
                        {
                            "map_dir": map_dir,
                            "waypoints_csv": waypoints_csv,
                            "corridor_csv": corridor_csv,
                            "speed_limit_override_kmph": ParameterValue(
                                LaunchConfiguration("speed_limit_kmph"),
                                value_type=float,
                            ),
                            "goal_mode": LaunchConfiguration("goal_mode"),
                            "use_sim_time": use_sim_time,
                        },
                        optional=("goal_mode",),
                    ),
                ],
            ),
            Node(
                package="krowd_stanley_control",
                executable="vehicle_state_node",
                name="vehicle_state_node",
                output="screen",
                parameters=[
                    vehicle_config,
                    tracking_config,
                    NodeParamsFile(
                        "vehicle_state_node",
                        {
                            "use_sim_time": use_sim_time,
                            "primary_odom_topic": LaunchConfiguration(
                                "primary_odom_topic"
                            ),
                            "fallback_odom_topic": LaunchConfiguration(
                                "fallback_odom_topic"
                            ),
                            "output_frame": LaunchConfiguration(
                                "vehicle_state_frame"
                            ),
                        },
                    ),
                ],
            ),
            Node(
                package="krowd_stanley_control",
                executable="stanley_node",
                name="stanley_node",
                output="screen",
                condition=_controller_selected(
                    launch_controller, controller_type, "stanley"
                ),
                parameters=[
                    vehicle_config,
                    tracking_config,
                    NodeParamsFile(
                        "stanley_node",
                        {
                            "use_sim_time": use_sim_time,
                            "angle_topic": LaunchConfiguration("angle_topic"),
                            "speed_topic": LaunchConfiguration("speed_topic"),
                            "brake_topic": LaunchConfiguration("brake_topic"),
                            "steering_feedback_topic": LaunchConfiguration(
                                "steering_feedback_topic"
                            ),
                        },
                    ),
                ],
                remappings=[
                    ("/target_angle", LaunchConfiguration("angle_topic")),
                    ("/target_speed", LaunchConfiguration("speed_topic")),
                    ("/brake", LaunchConfiguration("brake_topic")),
                ],
            ),
            Node(
                package="krowd_stanley_control",
                executable="avoidance_supervisor_node",
                name="avoidance_supervisor_node",
                output="screen",
                condition=_controller_selected(
                    launch_controller, controller_type, "stanley"
                ),
                parameters=[
                    tracking_config,
                    NodeParamsFile(
                        "avoidance_supervisor_node",
                        {
                            "use_sim_time": use_sim_time,
                            "launch_command": LaunchConfiguration("cone_launch_command"),
                            "command_topic": LaunchConfiguration("speed_topic"),
                        },
                    ),
                ],
            ),
        ]
    )
