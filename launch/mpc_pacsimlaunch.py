import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent, DeclareLaunchArgument, OpaqueFunction
from launch.events import Shutdown


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition

def getFullFilePath(name, dir, package='pacsim'):
    return os.path.join(get_package_share_directory(package), dir, name)


def generate_launch_description():
    # PACSim configuration
    track_frame = "map"
    realtime_ratio = 1.0
    xacro_file_name = 'separate_model.xacro'
    xacro_path = getFullFilePath(xacro_file_name, "urdf")

    # Launch arguments
    track_name_arg = DeclareLaunchArgument(
        "track_name",
        default_value="FSE23_centerline.yaml",
        description="Track file name in pacsim tracks dir (e.g. FSE23_centerline.yaml, FSG21_centerline.yaml, FSE24_centerline.yaml) or full path"
    )
    discipline_arg = DeclareLaunchArgument(
        "discipline",
        default_value="trackdrive",
        description="FS discipline: 'autocross' (3 laps) or 'trackdrive' (10 laps)"
    )
    centerline_topic_arg = DeclareLaunchArgument(
        "centerline_topic",
        default_value="/pacsim/track/centerline_smoothed",
        description="Reference centerline topic: /pacsim/track/centerline_smoothed (full closed circuit) or /pacsim/track/centerline_smoothed_front"
    )
    log_dir_arg = DeclareLaunchArgument(
        "log_dir",
        default_value="/workspace/MPC_logs",
        description="MPC logger output directory"
    )
    default_mpc_params_path = os.path.join(
        get_package_share_directory('etdv_mpc'), 'config', 'mpc_params.yaml'
    )
    mpc_params_arg = DeclareLaunchArgument(
        "mpc_params",
        default_value=default_mpc_params_path,
        description="Path to mpc_params.yaml"
    )
    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Whether to launch Foxglove Bridge for visualization"
    )

    discipline = LaunchConfiguration("discipline")
    centerline_topic = LaunchConfiguration("centerline_topic")
    log_dir = LaunchConfiguration("log_dir")
    mpc_params = LaunchConfiguration("mpc_params")

    # Foxglove Bridge Node / Launch (only started if use_foxglove:=true)
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        ),
        condition=IfCondition(LaunchConfiguration("use_foxglove"))
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'publish_frequency': float(50),
             'robot_description': Command(['xacro', ' ', xacro_path])}
        ],
        arguments=[xacro_path]
    )

    # Dynamic PACSim Simulator & MPC Controller Nodes
    def create_simulation_and_mpc_nodes(context, *args, **kwargs):
        track_val = LaunchConfiguration("track_name").perform(context).strip()
        if os.path.isabs(track_val):
            resolved_track_path = track_val
        else:
            resolved_track_path = getFullFilePath(track_val, "tracks")

        resolved_discipline = discipline.perform(context).strip()
        mpc_params_path = mpc_params.perform(context).strip()
        resolved_log_dir = log_dir.perform(context).strip()
        resolved_centerline_topic = centerline_topic.perform(context).strip()

        # PACSim Simulator Node
        nodePacsim = Node(
            package='pacsim',
            namespace='pacsim',
            executable='pacsim_node',
            name='pacsim_node',
            parameters=[
                {'use_sim_time': True}, 
                {"track_name": resolved_track_path}, 
                {"grip_map_path": getFullFilePath("gripMap.yaml", "tracks")}, 
                {"track_frame": track_frame}, 
                {"realtime_ratio": realtime_ratio}, 
                {"report_file_dir": "/tmp"}, 
                {"main_config_path": getFullFilePath("mainConfig.yaml", dir="config")}, 
                {"perception_config_path": getFullFilePath("perception.yaml", dir="config")}, 
                {"sensors_config_path": getFullFilePath("sensors.yaml", dir="config")}, 
                {"vehicle_model_config_path": getFullFilePath("vehicleModel.yaml", dir="config")}, 
                {"lidar_config_path": getFullFilePath("lidar.yaml", dir="config")}, 
                {"discipline": resolved_discipline}
            ],
            output="screen",
            emulate_tty=True
        )

        # Shutdown handler
        nodePacsimShutdownEventHandler = RegisterEventHandler(
            OnProcessExit(
                target_action=nodePacsim,
                on_exit=[
                    LogInfo(msg=('PACSim closed')),
                    EmitEvent(event=Shutdown(reason='PACSim closed, shutting down.')),
                ]
            )
        )

        parameters = []
        if mpc_params_path and os.path.isfile(mpc_params_path):
            parameters.append(mpc_params_path)
        else:
            parameters.append({
                'use_sim_time': True,
                'control_rate': 100.0,
                'mpc_dt': 0.05,
                'max_torque_per_wheel': 100.0,
                'outer_steering_ratio': 0.23,
                'max_lateral_error': 3.0,
                'emergency_stop': False,
                'default_track_width': 3.0,
                'track_margin': 0.90,
                'effective_mu': 1.0,
                'max_accel': 3.5,
                'min_accel': -8.0,
                'stop_on_trajectory_complete': False
            })

        overrides = {
            'use_sim_time': True,
        }
        if resolved_log_dir:
            overrides['log_dir'] = resolved_log_dir
        if resolved_centerline_topic:
            overrides['centerline_topic'] = resolved_centerline_topic

        parameters.append(overrides)

        mpc_node = Node(
            package='etdv_mpc',
            executable='mpc_pacsim_node',
            name='mpc_controller',
            output='screen',
            emulate_tty=True,
            parameters=parameters
        )

        return [nodePacsim, nodePacsimShutdownEventHandler, mpc_node]

    return LaunchDescription([
        track_name_arg,
        discipline_arg,
        centerline_topic_arg,
        log_dir_arg,
        mpc_params_arg,
        use_foxglove_arg,
        foxglove_bridge_launch,
        robot_state_publisher,
        OpaqueFunction(function=create_simulation_and_mpc_nodes)
    ])
