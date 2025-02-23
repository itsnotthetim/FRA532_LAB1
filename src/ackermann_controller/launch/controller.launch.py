from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    controller = LaunchConfiguration('controller')
    fk_model = LaunchConfiguration('fk_model')
    ik_model = LaunchConfiguration('ik_model')

    package_name = "ackermann_controller"

    ackermann_fk = Node(
        package=package_name,
        executable="ackermann_fk.py",
        name="ackermann_fk",
        output="screen",
        parameters=[
            {"kinematic_model": fk_model},
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
    launch_description.add_action(ackermann_fk)
    launch_description.add_action(ackermann_ik)
    # launch_description.add_action(pid_controller)
    # launch_description.add_action(pure_pursuit_controller)

    return launch_description