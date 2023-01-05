import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            
            composable_node_descriptions=[
                ComposableNode(
                    package='ros2_usbcapture',
                    plugin='wmj::camera_node',
                    name='talker',
                    extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ComposableNode(
                    package='deepdetector',
                    plugin='wmj::DeepDetector_Node',
                    name='listener',
                    extra_arguments=[{'use_intra_process_comms': True}]
                    )
            ],
            output='screen',
    )
    topaimer_node = Node(
        package="top_aimer",
        executable="top_aimer_node"
        )
    return launch.LaunchDescription([container,topaimer_node])
