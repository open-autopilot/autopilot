# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import SetRemap


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    autopilot_bringup_dir = get_package_share_directory('autopilot_bringup')
    launch_dir = os.path.join(autopilot_bringup_dir, 'launch')
    params_dir = os.path.join(autopilot_bringup_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_autopilot_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # robot_localization_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'autopilot_ekf.launch.py'))
    # )

    navigation2_cmd = GroupAction(actions=[ 
            SetRemap(src='/cmd_vel',dst='/robot/cmd_vel_aut'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "params_file": configured_params,
                    "autostart": "True",
                }.items(),
            )
        ]
    )
   
    # Create the launch description and populate
    ld = LaunchDescription()
    # ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)

    return ld
