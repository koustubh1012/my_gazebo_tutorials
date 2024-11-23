from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Function to generate launch description
def generate_launch_description():
    
    # Declare record_bag argument to control ros bag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',  # Default to not recording if not specified
        description='Enable or disable ros bag recording'
    )
    
    # create handle for walker node
    walker_node = Node(
        package='walker',
        executable='walker',
        name='walker',
        output='screen',
    )
    
    
    # create handle for conditional recording of ros bag
    def conditional_rosbag_record(context):
        if LaunchConfiguration('record_bag').perform(context) == 'true':
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '--all'],
                    output='screen',
                    cwd='src/walker/bag_files'  # Set the working directory
                )
            ]
        return []

    # return launch description
    return LaunchDescription([
        record_bag_arg,
        walker_node,
        OpaqueFunction(function=conditional_rosbag_record)
    ])