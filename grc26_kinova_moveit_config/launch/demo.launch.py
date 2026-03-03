from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    use_sim_time = {'use_sim_time': True}
    
    moveit_config = (
             MoveItConfigsBuilder("gen3_2f_85_pick_place", package_name="grc26_kinova_moveit_config")
            .robot_description()
            .planning_scene_monitor(
                publish_robot_description=True, publish_robot_description_semantic=True
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .to_moveit_configs()
         )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time] 
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory("grc26_kinova_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
            use_sim_time,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    kinova1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kinova1_controller", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    kinova2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kinova2_controller", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    g1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["g1_controller", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    g2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["g2_controller", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    rviz_config_path = os.path.join(
        get_package_share_directory("grc26_kinova_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )
    return LaunchDescription(
        [
            move_group_node,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            kinova1_controller_spawner,
            kinova2_controller_spawner,
            g1_controller_spawner,
            g2_controller_spawner,
            rviz_node,
        ]
    )


