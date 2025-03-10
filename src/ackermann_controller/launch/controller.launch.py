from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    controller = LaunchConfiguration('controller')
    fk_model = LaunchConfiguration('fk_model')
    ik_model = LaunchConfiguration('ik_model')

    package_name = "ackermann_controller"

    ground_truth_fk = Node(
        package=package_name,
        executable="ackermann_fk.py",
        namespace="ground_truth",
        output="screen",
        parameters=[
            {"kinematic_model": "ground_truth"},
            {"pub_tf": False},
        ],
    )

    single_track_fk = Node(
        package=package_name,
        executable="ackermann_fk.py",
        namespace="single_track",
        output="screen",
        parameters=[
            {"kinematic_model": "single_track"},
            {"pub_tf": False},
        ],
    )

    double_track_fk = Node(
        package=package_name,
        executable="ackermann_fk.py",
        namespace="double_track",
        output="screen",
        parameters=[
            {"kinematic_model": "double_track"},
            {"pub_tf": True},           
        ],
    )

    yaw_rate_fk = Node(
        package=package_name,
        executable="ackermann_fk.py",
        namespace="yaw_rate",
        output="screen",
        parameters=[
            {"kinematic_model": "yaw_rate"},
            {"pub_tf": False},
        ],
    )

    ackermann_ik = Node(
        package=package_name,
        executable="ackermann_ik.py",
        name="ackermann_ik",
        output="screen",
        parameters=[
            {"model": ik_model},
        ],
    )

    gps_emulator = Node(
        package=package_name,
        executable='gps_emulator.py',
        output='screen'
    )

    ekf_node = Node(
        package=package_name,
        executable='ekf_node.py',
        output='screen'
    )

    pid_controller = Node(
        package=package_name,
        executable='pid_controller.py',
        name='pid_controller',
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'pid'"])
        )
    )

    pure_pursuit_controller = Node(
        package=package_name,
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_controller',
        condition=IfCondition(
            PythonExpression(["'", controller, "' == 'pure_pursuit'"])
        )
    )

    data_saver = Node(
        package=package_name,
        executable='data_saver.py',
        output='screen'
    )

    validate_node = Node(
        package=package_name,
        executable='kinematic_validate_node.py',
        output='screen'
    )

    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="pid",
        description="Which controller to use",
    )

    declare_fk_model = DeclareLaunchArgument(
        "fk_model",
        default_value="ground_truth",
        description="Which model to use for forward kinematics",
    )

    declare_ik_model = DeclareLaunchArgument(
        "ik_model",
        default_value="ackermann",
        description="Which model to use for inverse kinematics",
    )

    launch_description = LaunchDescription()
    launch_description.add_action(declare_controller)
    launch_description.add_action(declare_fk_model)
    launch_description.add_action(declare_ik_model)
    launch_description.add_action(ackermann_ik)
    launch_description.add_action(ground_truth_fk)
    launch_description.add_action(single_track_fk)
    launch_description.add_action(double_track_fk)
    launch_description.add_action(yaw_rate_fk)
    launch_description.add_action(gps_emulator)
    launch_description.add_action(ekf_node)
    launch_description.add_action(data_saver)

    return launch_description