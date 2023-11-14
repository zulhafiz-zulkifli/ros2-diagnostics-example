import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node

useNamespace = False
px400v_namespace = 'agv'

def generate_launch_description():

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px400v_base'),
                'launch/main.launch.py'
            )
        )
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px400v_description'),
                'launch/main.launch.py'
            )
        )
    )

    hmi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px400v_hmi'),
                'launch/main.launch.py'
            )
        )
    )

    interfaces_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px400v_interfaces'),
                'launch/main.launch.py'
            )
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px400v_navigation'),
                'launch/main.launch.py'
            )
        )
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px400v_base'),
                'launch/cameras.launch.py'
            )
        )
    )

    nodes = []

    ## Biovet Main node
    # biovet_main_node = Node(
    #                     package = 'px400v_master',
    #                     executable = 'biovet_main.py',
    #                     namespace = ''
    #                    )
    
    ## Dock action server
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'dock_action_server.py',
                namespace = ''
        )
    )

    ## Refill food action server
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'refill_food_action_server.py',
                namespace = ''
        )
    )

    ## Dispense food service server
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'dispense_food_service.py',
                namespace = ''
        )
    )

    ## Feeding action server
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'feeding_action_server.py',
                namespace = ''
        )
    )

    ## Disinfect action server
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'disinfect_action_server.py',
                namespace = ''
        )
    )

    ## PLC EIP node
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'plc_eip.py',
                namespace = ''
        )
    )

    ## PLC EIP Heartbeat node
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'plc_eip_heartbeat.py',
                namespace = ''
        )
    )

    ## BFS Main node
    nodes.append(
        Node(
                package = 'px400v_master',
                executable = 'bfs_main.py',
                namespace = ''
        )
    )

    base_launch_ns = GroupAction(actions =[ PushRosNamespace(px400v_namespace), base_launch])

    description_launch_ns = GroupAction(actions = [PushRosNamespace(px400v_namespace), description_launch])

    hmi_launch_ns = GroupAction(actions = [PushRosNamespace(px400v_namespace), hmi_launch])

    interfaces_launch_ns = GroupAction(actions = [PushRosNamespace(px400v_namespace), interfaces_launch])

    navigation_launch_ns = GroupAction(actions = [PushRosNamespace(px400v_namespace), navigation_launch])

    if useNamespace:
        return LaunchDescription([
            base_launch_ns,
            description_launch_ns,
            hmi_launch_ns,
            interfaces_launch_ns,
            navigation_launch_ns
        ])
    else:
        return LaunchDescription([
            base_launch,
            navigation_launch,
            # cameras_launch,
            # description_launch,
            # hmi_launch,
            # interfaces_launch,
        ] + nodes)