from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='work_bt',
            executable='excavator_bt',
            name='excavator_bt',
            output='screen',
            parameters=[
                {'bt_xml': PathJoinSubstitution([
                    FindPackageShare('work_bt'),
                    'config',
                    'excavator_bt.xml'])}
            ]
        )
    ])
