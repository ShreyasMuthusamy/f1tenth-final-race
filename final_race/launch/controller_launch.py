import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   ld = LaunchDescription()
   
   controller = Node(
       package='final_race',
       executable='controller_node.py',
       name='controller_node',
   )
   
   visualizer = Node(
       package='final_race',
       executable='waypoint_visualizer.py',
       name='waypoint_visualizer',
   )
   
   ld.add_action(controller)
   ld.add_action(visualizer)
   return ld

