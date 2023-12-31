#!/usr/bin/env python3
# Copyright 2021 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

# from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    analyzer_params_filepath = PathJoinSubstitution(
        [get_package_share_directory('diagnostics_example'), 'config', 'diagnostics.yaml'])

    # namespaced_param_file = RewrittenYaml(
    #     source_file=analyzer_params_filepath,
    #     root_key=LaunchConfiguration('namespace'),
    #     param_rewrites={},
    #     convert_types=True)

    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath],
        remappings=[
            ('/diagnostics', 'diagnostics'),
            ('/diagnostics_agg', 'diagnostics_agg'),
            ('/diagnostics_toplevel_state', 'diagnostics_toplevel_state')
        ])
    diag_publisher = Node(
         package='diagnostics_example',
         executable='diagnostics_updater.py',
         output='screen',
         remappings=[
            ('/diagnostics', 'diagnostics'),
            ('/diagnostics_agg', 'diagnostics_agg'),
            ('/diagnostics_toplevel_state', 'diagnostics_toplevel_state')]
        )

    cpu_monitor = Node(
         package='diagnostics_example',
         executable='computer_monitor.py',
         output='screen'
        )

    return launch.LaunchDescription([
        aggregator,
        diag_publisher,
        cpu_monitor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=aggregator,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
