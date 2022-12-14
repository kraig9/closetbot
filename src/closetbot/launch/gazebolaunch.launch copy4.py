# -*- coding: utf-8 -*-
import os
from platform import node

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    # Robot specific arguments
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="closetbot",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="/home/kraig9/ws_moveit2/src/closetbot/urdf/closetbot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed, then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="/home/kraig9/ws_moveit2/src/closetbot/urdf/closetbot.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="/home/kraig9/ws_moveit2/src/closetbot/ros_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    closetbot = "closetbot"
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_controller = LaunchConfiguration("robot_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            "/home/kraig9/ws_moveit2/src/closetbot/urdf/closetbot.urdf.xacro",
            " ",
            "name:=",
            closetbot,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # robot_controllers = "/home/kraig9/ws_moveit2/src/closetbot/config/arm_controller.yaml"
    robot_controllers = "/home/kraig9/ws_moveit2/src/closetbot/config/ros_controllers.yaml"
    # robot_controllers = "/home/kraig9/ws_moveit2/src/closetbot/config/controllers.yaml"
    # PathJoinSubstitution(
    #     # [FindPackageShare(description_package), "config", controllers_file]
    # )

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            "/home/kraig9/ws_moveit2/src/closetbot/urdf/closetbot.srdf.xacro"
            # PathJoinSubstitution(
            #     [FindPackageShare(description_package), "urdf", moveit_config_file]
            # ),
            " ",
            "name:=",
            closetbot,
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Load kinematics yaml
    kinematics_yaml = load_yaml("closetbot", "/home/kraig9/ws_moveit2/src/closetbot/config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # TODO(destogl): change this hard-coded name to "description_package"
    ompl_planning_yaml = load_yaml(
        "closetbot", "/home/kraig9/ws_moveit2/src/closetbot/config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    # TODO(destogl): check how to use ros2_control's controllers-yaml
    # controllers_yaml = load_yaml("closetbot", "/home/kraig9/ws_moveit2/src/closetbot/config/controllers.yaml")
    # controllers_yaml = load_yaml("closetbot", "/home/kraig9/ws_moveit2/src/closetbot/config/arm_controller.yaml")
    controllers_yaml = load_yaml("closetbot", "/home/kraig9/ws_moveit2/src/closetbot/config/controllers.yaml")
    print (controllers_yaml)
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    # rviz with moveit configuration
    rviz_config_file = "/home/kraig9/ws_moveit2/src/closetbot/rviz/view_robot.rviz"

    rviz_node = Node(
        package="rviz2",
        # condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )


    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    cam_params = {"video_device": "/dev/video0", "camera_name" : "usb_cam", "camera_info_url" : "file:///home/kraig9/ws_moveit2/src/closetbot/config/camera/usb_cam.yaml"}
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[cam_params],
    )

    cam2_params = {"video_device": "/dev/video1", "camera_name" : "usb_cam2", "camera_info_url" : "file:///home/kraig9/ws_moveit2/src/closetbot/config/camera/usb_cam2.yaml"}
    camera2_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[cam2_params],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
            # "--controller-manager",
            # "/controller_manager",
        ],
    )


    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', 'joint_state_controller'],
    #     output='screen'
    # )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_trajectory_controller'],
        output='screen'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(closetbot),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                # PythonLaunchDescriptionSource([os.path.join(
                #     get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                PythonLaunchDescriptionSource('/opt/ros/galactic/share/gazebo_ros/launch/gazebo.launch.py'),
                launch_arguments={'verbose' : 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # gazebo_spawn_entity = Node(package='gazebo_ros',
    # executable='spawn_entity.py',
    # gazebo_spawn_entity = Node(
    #                     package='gazebo_ros',
    #                     executable='/opt/ros/galactic/lib/gazebo_ros/spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'closetbot'],
    #                     output='screen')
    gazebo_spawn_entity = Node(package='gazebo_ros',
    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'closetbot'],
                    output='screen')

                        # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link1'])

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_position_controller", "-c", "/controller_manager"],
    # )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[control_node],
        )
    )
    delay_move_group_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[move_group_node],
        )
    )

    delay_joint_state_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_move_group_node_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[move_group_node],
        )
    )

    # delay_spawn_entity_after_gazebo = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=gazebo,
    #         on_exit=[gazebo_spawn_entity],
    #     )
    # )

    nodes_to_start = [
        # control_node,
        # delay_joint_state_broadcaster_spawner_after_robot_controller_spawner,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        # rviz_node,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        move_group_node,
        # delay_move_group_node_after_joint_state_broadcaster_spawner,
        gazebo,
        # delay_spawn_entity_after_gazebo
        gazebo_spawn_entity
        # static_tf
    ]

    # event_handlers = [
    #     RegisterEventHandler(
    #       event_handler=OnProcessExit(
    #           target_action=gazebo_spawn_entity,
    #           on_exit=[load_joint_state_controller],
    #       )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_controller,
    #             on_exit=[load_joint_trajectory_controller],
    #         )
    #     )
    # ]

    return LaunchDescription(
        declared_arguments +
        nodes_to_start
        # event_handlers
    )
