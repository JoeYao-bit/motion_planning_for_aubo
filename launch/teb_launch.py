from launch import LaunchDescription
import launch_ros.actions

new_global_path_topic = '/global_path'

# from src to dest
# need colcon build to update remapping !!
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
            remappings=[('/global_path', new_global_path_topic)]
            ),
        launch_ros.actions.Node(
            namespace= "teb_planner", 
            package='teb_planner', 
            executable='fake_robot_node', 
            output='screen',
            remappings=[('/global_path', new_global_path_topic)]
            ),
        launch_ros.actions.Node(
            namespace= "teb_planner", 
            package='teb_planner', 
            executable='teb_node', 
            output='screen',
            # need colcon build to update config !!
            remappings=[("/global_path", new_global_path_topic)],
                        parameters=[
                {"local_expand_pixel": 20}, # local dist map expand from the bounding box of local path 
                {"path_forward_length": 1.4}, # only use partial path to compute velocity command
                {"max_dist_to_path": 0.5} # if current pose if far to global path, stop TEB
            ]
            )
    ])
