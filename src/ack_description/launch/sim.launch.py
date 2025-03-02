import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
import launch_ros.actions
import xacro

def generate_launch_description():
    package_name = "ack_description"
    controller_package_name = "ackermann_controller"
    rviz_file_name = "lab1-1_validate_config.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "load_world_into_gazebo.launch.py"
                )
            ]
        ),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "ackbot",
            "-x", "9.073496746393584",
            "-y", "-0.00005465073750971568",
            "-Y", "1.5700039414375448"
        ],
        output = "screen"
    )
    
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
    # )
    # position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["position_controller", "--controller-manager", "/controller_manager"],
    # )

    # joint_state_broadcaster = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
    #     output="screen"
    # )

    # forward_position_controllers = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active", "forward_position_controllers"],
    #     output="screen"
    # )

    # forward_velocity_controllers = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active", "forward_velocity_controllers"],
    #     output="screen"
    # )

    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen"
    )

    forward_position_controllers = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "position_controllers"],
        output="screen"
    )

    forward_velocity_controllers = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "velocity_controllers"],
        output="screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(controller_package_name),
                    "launch",
                    "controller.launch.py"
                )
            ]
        ),
        launch_arguments={"controller":"pid", "fk_model":"ground_truth", "ik_model":"ackermann"}.items()
    )

    launch_description = LaunchDescription()

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster, forward_velocity_controllers, forward_position_controllers],
            )
        )
    )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster,
    #             on_exit=[forward_velocity_controllers],
    #         )
    #     )
    # )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=forward_velocity_controllers,
    #             on_exit=[forward_position_controllers],
    #         )
    #     )
    # )


    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=forward_position_controllers,
                on_exit=[rviz, controller]
            )
        )
    )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_entity,
    #             on_exit=[joint_state_broadcaster_spawner],
    #         )
    #     )
    # )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[robot_controller_spawner, position_controller_spawner],
    #         )
    #     )
    # )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[rviz, controller],
    #         )
    #     )
    # )

    # Static Transform Publisher (world -> odom)
    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        output="screen"
    )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(rsp)
    launch_description.add_action(static_tf)
    return launch_description