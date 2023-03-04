import os
import yaml
from yaml import Loader
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# from ur_moveit_config.launch_common import load_yaml

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
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
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

  declared_arguments = []
  # UR specific arguments
  declared_arguments.append(
      DeclareLaunchArgument("ur_type", description="Type/series of used UR robot.")
  )
  # TODO(anyone): enable this when added into ROS2-foxy
  # choices=['ur3', 'ur3e', 'ur5', 'ur5e', 'ur10', 'ur10e', 'ur16e']))
  declared_arguments.append(
      DeclareLaunchArgument(
          "robot_ip", description="IP address by which the robot can be reached."
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "safety_limits",
          default_value="true",
          description="Enables the safety limits controller if true.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "safety_pos_margin",
          default_value="0.15",
          description="The margin to lower and upper limits in the safety controller.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "safety_k_position",
          default_value="20",
          description="k-position factor in the safety controller.",
      )
  )
  # General arguments
  declared_arguments.append(
      DeclareLaunchArgument(
          "runtime_config_package",
          default_value="ur_bringup",
          description='Package with the controller\'s configuration in "config" folder. \
      Usually the argument is not set, it enables use of a custom setup.',
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "controllers_file",
          default_value="ur_controllers.yaml",
          description="YAML file with the controllers configuration.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "description_package",
          default_value="ur_description",
          description="Description package with robot URDF/XACRO files. Usually the argument \
      is not set, it enables use of a custom description.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "description_file",
          default_value="ur.urdf.xacro",
          description="URDF/XACRO description file with the robot.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "prefix",
          default_value='""',
          description="Prefix of the joint names, useful for \
      multi-robot setup. If changed than also joint names in the controllers' configuration \
      have to be updated.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "use_fake_hardware",
          default_value="false",
          description="Start robot with fake hardware mirroring command to its states.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "fake_sensor_commands",
          default_value="false",
          description="Enable fake command interfaces for sensors used for simple simulations. \
          Used only if 'use_fake_hardware' parameter is true.",
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
      DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
  )

  declared_arguments.append(
      DeclareLaunchArgument(
          "moveit_config_package",
          default_value="ur_moveit_config",
          description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
      is not set, it enables use of a custom moveit config.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "moveit_config_file",
          default_value="ur.srdf.xacro",
          description="MoveIt SRDF/XACRO description file with the robot.",
      )
  )
  # Initialize Arguments
  ur_type = LaunchConfiguration("ur_type")
  robot_ip = LaunchConfiguration("robot_ip")
  safety_limits = LaunchConfiguration("safety_limits")
  safety_pos_margin = LaunchConfiguration("safety_pos_margin")
  safety_k_position = LaunchConfiguration("safety_k_position")
  # General arguments
  runtime_config_package = LaunchConfiguration("runtime_config_package")
  controllers_file = LaunchConfiguration("controllers_file")
  description_package = LaunchConfiguration("description_package")
  description_file = LaunchConfiguration("description_file")
  prefix = LaunchConfiguration("prefix")
  use_fake_hardware = LaunchConfiguration("use_fake_hardware")
  fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
  robot_controller = LaunchConfiguration("robot_controller")
  launch_rviz = LaunchConfiguration("launch_rviz")
  moveit_config_package = LaunchConfiguration("moveit_config_package")
  moveit_config_file = LaunchConfiguration("moveit_config_file")


  joint_limit_params = PathJoinSubstitution(
      [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
  )
  kinematics_params = PathJoinSubstitution(
      [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
  )
  physical_params = PathJoinSubstitution(
      [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
  )
  visual_params = PathJoinSubstitution(
      [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
  )
  script_filename = PathJoinSubstitution(
      [FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"]
  )
  input_recipe_filename = PathJoinSubstitution(
      [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
  )
  output_recipe_filename = PathJoinSubstitution(
      [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
  )

  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
          " ",
          "robot_ip:=",
          robot_ip,
          " ",
          "joint_limit_params:=",
          joint_limit_params,
          " ",
          "kinematics_params:=",
          kinematics_params,
          " ",
          "physical_params:=",
          physical_params,
          " ",
          "visual_params:=",
          visual_params,
          " ",
          "safety_limits:=",
          safety_limits,
          " ",
          "safety_pos_margin:=",
          safety_pos_margin,
          " ",
          "safety_k_position:=",
          safety_k_position,
          " ",
          "name:=",
          ur_type,
          " ",
          "script_filename:=",
          script_filename,
          " ",
          "input_recipe_filename:=",
          input_recipe_filename,
          " ",
          "output_recipe_filename:=",
          output_recipe_filename,
          " ",
          "prefix:=",
          prefix,
          " ",
          "use_fake_hardware:=",
          use_fake_hardware,
          " ",
          "fake_sensor_commands:=",
          fake_sensor_commands,
          " ",
      ]
  )
  robot_description = {"robot_description": robot_description_content}

  robot_controllers = PathJoinSubstitution(
      [FindPackageShare(runtime_config_package), "config", controllers_file]
  )

  rviz_config_file = PathJoinSubstitution(
      [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
  )

  # MoveIt Configuration
  robot_description_semantic_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
          ),
          " ",
          "name:=",
          # Also ur_type parameter could be used but then the planning group names in yaml
          # configs has to be updated!
          "ur",
          " ",
          "prefix:=",
          prefix,
          " ",
      ]
  )
  robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

  kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
  robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

  # Planning Configuration
  ompl_planning_pipeline_config = {
      "move_group": {
          "planning_plugin": "ompl_interface/OMPLPlanner",
          "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
          "start_state_max_bounds_error": 0.1,
      }
  }
  ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
  ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

  # Trajectory Execution Configuration
  controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
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

  moveit_cpp_config = yaml.load("""
        planning_scene_monitor_options:
          name: "planning_scene_monitor"
          robot_description: "robot_description"
          joint_state_topic: "/joint_states"
          attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
          publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
          monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
          wait_for_initial_state_timeout: 10.0

        planning_pipelines:
          #namespace: "moveit_cpp"  # optional, default is ~
          pipeline_names: ["ompl"]

        plan_request_params:
          planning_time: 10.0
          planner_id: "this_is_stupid"
          planning_attempts: 3
          planning_pipeline: ompl
          max_velocity_scaling_factor: 0.5
          max_acceleration_scaling_factor: 0.5

        # octomap parameters (when used)
        octomap_frame: base_link
        octomap_resolution: 0.01
        max_range: 5.0""", Loader=Loader)

## Additions for running on hardware ## 

## moved these up for cleaner definition 
  myworkcell_node = Node(
        name='myworkcell_node',
        package='myworkcell_core',
        executable='myworkcell_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
                {
                    'base_frame': 'base_link',
#                    'robot_description_planning' : joint_limits_config,
                    'planning_pipelines': ['ompl'],
                },
            moveit_cpp_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            ],
        )

  fake_ar_publisher_node = Node(
        name='fake_ar_publisher_node',
        package='fake_ar_publisher',
        executable='fake_ar_publisher_node',
        output='screen',
    )
  vision_node = Node(
        name='vision_node',
        package='myworkcell_core',
        executable='vision_node',
        output='screen',
    )
        
  return LaunchDescription([
      myworkcell_node,
      fake_ar_publisher_node,
      vision_node
    ] + declared_arguments)
