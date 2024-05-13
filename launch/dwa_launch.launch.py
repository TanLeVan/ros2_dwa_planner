from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	config_file = os.path.join(get_package_share_directory('ros2_dwa_planner'),
		'config',
        'config.yaml'
        )
	return LaunchDescription([
		Node(
			package = "ros2_dwa_planner",
			executable = "goal_publisher",
			name = "goal_publisher",
			parameters = [config_file]
        ),
		Node(
			package = "ros2_dwa_planner",
			executable = "footprint_publisher",
			name = "goal_publisher",
			parameters = [config_file]
        ),
		Node(
			package = "ros2_dwa_planner",
			executable = "dwa_planner_node",
			name = "goal_publisher",
			parameters = [config_file]
        )
	])