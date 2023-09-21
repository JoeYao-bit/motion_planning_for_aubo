from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "teb_planner", package='teb_planner', executable='generate_global_path', output='screen'),
        launch_ros.actions.Node(
            namespace= "teb_planner", package='teb_planner', executable='fake_robot_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "teb_planner", package='teb_planner', executable='teb_node', output='screen')
    ])
