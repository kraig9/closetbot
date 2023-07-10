import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_file2(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "safety_limits",
    #         default_value="true",
    #         description="Enables the safety limits controller if true.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "safety_pos_margin",
    #         default_value="0.15",
    #         description="The margin to lower and upper limits in the safety controller.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "safety_k_position",
    #         default_value="20",
    #         description="k-position factor in the safety controller.",
    #     )
    # )
    # General arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_package",
    #         default_value="ur_e_description",
    #         description="Description package with robot URDF/XACRO files. Usually the argument \
    #     is not set, it enables use of a custom description.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="/home/closetbot/src/closetbot/urdf/DexArmHardware.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="/home/closetbot/src/closetbot/urdf/DexArmHardware.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "prefix",
    #         default_value='""',
    #         description="Prefix of the joint names, useful for \
    #     multi-robot setup. If changed than also joint names in the controllers' configuration \
    #     have to be updated.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    # )

    # Initialize Arguments
    # safety_limits = LaunchConfiguration("safety_limits")
    # safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    # safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    # prefix = LaunchConfiguration("prefix")
    # launch_rviz = LaunchConfiguration("launch_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            "/home/closetbot/src/closetbot/urdf/DexArm.urdf.xacro"
            " ",
            "name:=",
            # Also, ur_type parameter could be used, but then the planning group names in YAML
            # configs have to be updated!
            "DexArm_unofficial",
            " "
        ]
    )
    robot_description = {"robot_description": robot_description_content}

        # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            "/home/closetbot/src/closetbot/urdf/DexArm.srdf.xacro"
            " ",
            "name:=",
            "DexArm_unofficial",
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Load kinematics yaml
    kinematics_yaml = load_yaml("closetbot", "/home/closetbot/src/closetbot/config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="test_trajectory2",
        package="moveit2_scripts",
        executable="test_trajectory2",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            robot_description,
            robot_description_semantic, 
            kinematics_yaml,
            ],
    )

    nodes_to_start = [move_group_demo]

    return LaunchDescription(declared_arguments + nodes_to_start)