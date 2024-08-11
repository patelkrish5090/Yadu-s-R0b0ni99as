import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='12.994061')
    y_pose = LaunchConfiguration('y_pose', default='14.233000')
    
    world = os.path.join(
        get_package_share_directory('hackathon_automation'),
        'worlds',
        'MAP.world'
    )
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': '0.000044'
        }.items()
    )
    
    static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    
    set_env_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', 
        (os.path.join(get_package_share_directory('hackathon_automation'), 'models') + ':/home/.gazebo/models')
    )
    
    # Define the nodes with a delay
    # delayed_path_planner_node = TimerAction(
    #     period=10.0,
    #     actions=[
    #         Node(
    #             package='hackathon_automation',
    #             executable='path_planner.py',
    #             name='path_planner',
    #             output='screen',
    #             parameters=[{'use_sim_time': use_sim_time}]
    #         )
    #     ]
    # )

    # delayed_controller_node = TimerAction(
    #     period=30.0,
    #     actions=[
    #         Node(
    #             package='hackathon_automation',
    #             executable='controller.py',
    #             name='controller',
    #             output='screen',
    #             parameters=[{'use_sim_time': use_sim_time}]
    #         )
    #     ]
    # )

    # delayed_colour_detection_node = TimerAction(
    #     period=20.0,
    #     actions=[
    #         Node(
    #             package='hackathon_automation',
    #             executable='colour_detection.py',
    #             name='colour_detection',
    #             output='screen',
    #             parameters=[{'use_sim_time': use_sim_time}]
    #         )
    #     ]
    # )

    ld = LaunchDescription()

    ld.add_action(set_env_var)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(static_transform_publisher_cmd)

    # Add the delayed actions
    # ld.add_action(delayed_path_planner_node)
    # ld.add_action(delayed_controller_node)
    # ld.add_action(delayed_colour_detection_node)

    return ld
