from launch.launch_description import LaunchDescription
from launch.launch_service import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='publisher'),
        DeclareLaunchArgument('name', default_value='str_publisher'),
        DeclareLaunchArgument('publisher_topic', default_value='str_topic'),
        DeclareLaunchArgument('do_timer', default_value='True'),
        DeclareLaunchArgument('str_data', default_value='goblin'),
        ComposableNodeContainer(
            name='str_publisher_container',
            namespace='publisher',
            package='rclcpp_components',
            executable='component_container_mt',
            emulate_tty=True,
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    name=LaunchConfiguration('name'),
                    namespace=LaunchConfiguration('namespace'),
                    package='cpp_pub',
                    plugin='publisher::StrPublisher',
                    parameters=[{
                        'publisher_topic': LaunchConfiguration('publisher_topic'),
                        'do_timer': LaunchConfiguration('do_timer'),
                        'str_data': LaunchConfiguration('str_data')
                    }]
                )
            ]
        )
    ])
    

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()