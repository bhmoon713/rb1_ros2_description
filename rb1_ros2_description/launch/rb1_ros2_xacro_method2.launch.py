import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    description_package_name = "rb1_ros2_description"
    install_dir = get_package_prefix(description_package_name)

    # === Set Gazebo environment paths ===
    gazebo_models_path = os.path.join(description_package_name, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # === Launch Gazebo ===
    gazebo_launch_args = {
        'verbose': 'false',
        'pause': 'false',
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=gazebo_launch_args.items()
    )

    # === Robot Description and State Publisher ===
    robot_name_1 = ""
    robot_desc_file = "rb1_ros2_base.urdf.xacro"
    robot_desc_path = os.path.join(
        get_package_share_directory("rb1_ros2_description"),
        "xacro",
        robot_desc_file
    )

    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_1,
        parameters=[{
            'frame_prefix': robot_name_1 + '/',
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1]),
                value_type=str)
        }],
        output="screen"
    )

    # === Spawn robot in Gazebo ===
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_1,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-topic', robot_name_1 + '/robot_description'
        ],
        output='screen'
    )

    # === Controller Spawners ===
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rb1_base_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # === Delay controller spawners until after robot is spawned ===
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot1,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                position_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        rsp_robot1,
        spawn_robot1,
        delayed_controllers
    ])
