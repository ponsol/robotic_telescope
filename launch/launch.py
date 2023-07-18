import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch




def generate_launch_description():

    rviz_flag = DeclareLaunchArgument(
        "rviz_flag", default_value="False", description="Rviz flag"
    )


    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface -- possible values: [mock_components, uros]",
    )


    moveit_config = (
          MoveItConfigsBuilder("robotic_telescope", package_name="robotic_telescope")
          .robot_description(file_path="config/telescope.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
          )
          .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True 
            #,publish_planning_scene=True,	publish_geometry_updates = True
          )
          .trajectory_execution(file_path="config/moveit_controllers.yaml")
          .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
          )
          .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
             moveit_config.to_dict(),
             move_group_capabilities,
             #{"use_sim_time": True}
         ],
    )


    rviz_mode = LaunchConfiguration("rviz_flag")

    rviz_base = os.path.join(
        get_package_share_directory("robotic_telescope"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            #{"use_sim_time": True},
        ],
        condition=UnlessCondition(rviz_mode),
    )


    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_kinematics,
            #{"use_sim_time": True},
            ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("robotic_telescope"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    tel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["telgroup_controller", "-c", "/controller_manager"],
    )


    return LaunchDescription(
        [
            rviz_flag,
            ros2_control_hardware_type,
            rviz_node,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            tel_controller_spawner,
        ]
    )
