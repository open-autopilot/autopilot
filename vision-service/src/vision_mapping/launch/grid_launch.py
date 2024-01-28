import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_dir = get_package_share_directory('vision_mapping')
    param_file = os.path.join(params_dir, 'config/camera.yaml')   
    return LaunchDescription([
    	Node(
    	    package='vision_mapping',
    	    executable='listener',
    	    name='listener',
    	    remappings=[('/image/compressed', '/oak/rgb/image_raw/compressed')]
    	    ),
    	GroupAction(actions=[
   		IncludeLaunchDescription(
    			PythonLaunchDescriptionSource([
    			FindPackageShare("depthai_ros_driver"), '/launch', '/camera.launch.py']), 
	    		launch_arguments = {
	    			'params_file' : param_file
	    		}.items()
	    	)
    	])
   ])
