from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------------------------------------------------------------
    # Launch configuration (fixed values for this demo)
    # ---------------------------------------------------------------------
    # Note: this file currently uses direct Python string values instead of
    # launch arguments, so launching is simple and deterministic.
    # If you want runtime overrides, replace these with LaunchConfiguration
    # and DeclareLaunchArgument entries.
    description_package = "urdf_demo"
    description_file = "demo.urdf.xacro"
    tf_prefix = '""'

    # ---------------------------------------------------------------------
    # Build the robot_description by executing xacro at launch time
    # ---------------------------------------------------------------------
    # This command resolves to roughly:
    #   xacro <package_share>/urdf/demo.urdf.xacro tf_prefix:=<value>
    # The resulting XML string is passed to robot_state_publisher.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # RViz display configuration file used by rviz2.
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # ---------------------------------------------------------------------
    # Nodes
    # ---------------------------------------------------------------------
    # Publishes joint states with a GUI slider panel.
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Publishes the TF tree computed from robot_description + joint states.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Starts RViz with a predefined config to visualize robot model and TF.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Nodes started by this launch file.
    nodes_to_start = [
        #joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    # Return launch description with the selected nodes.
    return LaunchDescription(nodes_to_start)