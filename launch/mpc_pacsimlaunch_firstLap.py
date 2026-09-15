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

def getFullFilePath(name, dir, package='pacsim'):
    return os.path.join(get_package_share_directory(package), dir, name)


def generate_launch_description():
    # PACSim configuration
    track_name = "FSE23_centerline.yaml"
    track_frame = "map"
    realtime_ratio = 1.0
    discipline = "autocross"
    xacro_file_name = 'separate_model.xacro'
    xacro_path = getFullFilePath(xacro_file_name, "urdf")

    # Launch arguments
    log_dir_arg = DeclareLaunchArgument(
        "log_dir",
        default_value="/workspace/MPC_logs",
        description="MPC logger output directory"
    )
    mpc_params_arg = DeclareLaunchArgument(
        "mpc_params",
        default_value="",
        description="Optional path to custom mpc_params.yaml"
    )
    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Whether to launch Foxglove Bridge for visualization"
    )

    log_dir = LaunchConfiguration("log_dir")
    mpc_params = LaunchConfiguration("mpc_params")

    # Foxglove Bridge Node / Launch
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    # PACSim Simulator Node
    nodePacsim = Node(
        package='pacsim',
        namespace='pacsim',
        executable='pacsim_node',
        name='pacsim_node',
        parameters=[
            {'use_sim_time': True}, 
            {"track_name": getFullFilePath(track_name, "tracks")}, 
            {"grip_map_path": getFullFilePath("gripMap.yaml", "tracks")}, 
            {"track_frame": track_frame}, 
            {"realtime_ratio": realtime_ratio}, 
            {"report_file_dir": "/tmp"}, 
            {"main_config_path": getFullFilePath("mainConfig.yaml", dir="config")}, 
            {"perception_config_path": getFullFilePath("perception.yaml", dir="config")}, 
            {"sensors_config_path": getFullFilePath("sensors.yaml", dir="config")}, 
            {"vehicle_model_config_path": getFullFilePath("vehicleModel.yaml", dir="config")}, 
            {"lidar_config_path": getFullFilePath("lidar.yaml", dir="config")}, 
            {"discipline": discipline}
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

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'publish_frequency': float(1000),
             'robot_description': Command(['xacro', ' ', xacro_path])}
        ],
        arguments=[xacro_path]
    )

    # ETDV MPC Controller Node
    def create_mpc_node(context, *args, **kwargs):
        mpc_params_path = mpc_params.perform(context).strip()
        resolved_log_dir = log_dir.perform(context).strip()

        parameters = [{
            'use_sim_time': True,
            'control_rate': 100.0,
            'mpc_dt': 0.05,
            'max_torque_per_wheel': 100.0,
            'outer_steering_ratio': 0.23,
            'max_lateral_error': 3.0,
            'emergency_stop': False,
            'log_dir': resolved_log_dir,
            'default_track_width': 3.0,
            'track_margin': 0.85,
            'effective_mu': 1.0,
            'max_accel': 3.5,
            'min_accel': -8.0,
            'stop_on_trajectory_complete': False
        }]

        if mpc_params_path:
            parameters.insert(0, mpc_params_path)

        mpc_node = Node(
            package='etdv_mpc',
            executable='mpc_pacsim_node',
            name='mpc_controller',
            output='screen',
            emulate_tty=True,
            parameters=parameters
        )
        return [mpc_node]

    return LaunchDescription([
        log_dir_arg,
        mpc_params_arg,
        use_foxglove_arg,
        foxglove_bridge_launch,
        nodePacsim,
        nodePacsimShutdownEventHandler,
        robot_state_publisher,
        OpaqueFunction(function=create_mpc_node)
    ])
