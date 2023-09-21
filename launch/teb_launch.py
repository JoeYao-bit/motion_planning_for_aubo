from launch import LaunchDescription
import launch_ros.actions

global_path_topic = '/global_path12'

# from src to dest
# need colbuild to update remapping !!
# remappings=[
# 	('src1', 'dest1'),
# 	('src2', 'dest2'),
# 	...
# ]

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "teb_planner", 
            package='teb_planner', 
            executable='generate_global_path', 
            output='screen',
            remappings=[('/global_path', global_path_topic)]
            ),
        launch_ros.actions.Node(
            namespace= "teb_planner", 
            package='teb_planner', 
            executable='fake_robot_node', 
            output='screen',
            remappings=[('/global_path', global_path_topic)]
            ),
        launch_ros.actions.Node(
            namespace= "teb_planner", 
            package='teb_planner', 
            executable='teb_node', 
            output='screen',
            remappings=[("/global_path", global_path_topic)]
            )
    ])
