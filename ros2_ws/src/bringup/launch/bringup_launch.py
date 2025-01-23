from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='macro_sim',
            executable='macro_sim',
            name='macro_sim',
            output='screen',
            parameters=[]
        ),
        Node(
            package='rover_fsm',
            executable='rover_fsm',
            name='rover_fsm',
            output='screen',
            parameters=[]
        ),
        Node(
            package='rover',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
              PathJoinSubstitution([FindPackageShare('work_bt'), 'launch', 'excavator_bt_launch.py'])
            ])
        )
    ])
