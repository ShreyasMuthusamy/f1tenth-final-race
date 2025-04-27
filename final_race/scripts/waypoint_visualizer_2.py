#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import pandas as pd
# TODO CHECK: include needed ROS msg type headers and libraries

class WaypointVisualizer(Node):
    """ 
    Visualize the waypoints
    """
    def __init__(self):
        super().__init__('waypoint_viz_node')
        self.waypoints_visualizer = self.create_publisher(Marker, 'waypoints', 10)

        # wp_path = '/root/f1tenth_lab5/slam-and-pure-pursuit-team9/pure_pursuit/waypoints/interpolated_wp_w_speed.csv'
        # wp_path = '/home/nvidia/slam-and-pure-pursuit-team9/pure_pursuit/waypoints/drawn_traj_temp.csv'
        # wp_path = '/home/shubho/ese6150/sim_ws/src/pure_pursuit/waypoints/drawn_traj_temp.csv'
        # wp_path = "/root/f1tenth_lab5/slam-and-pure-pursuit-team9/pure_pursuit/waypoints/waypoint_w_speed_race2.csv"
        # wp_path = '/home/nvidia/f1tenth_ws/src/pure_pursuit/waypoints/race2/wp_w_speed_race2_3_235_anh.csv' ### working 
        
        # wp_path = '/home/nvidia/f1tenth_ws/src/pure_pursuit/waypoints/race2/wp_w_speed_race2_3_115_anh.csv'
        # wp_path = '/home/nvidia/f1tenth_ws/src/pure_pursuit/waypoints/race2/wp_w_speed_race2_2_873_2_8_prak.csv'
        # wp_path = '/home/nvidia/f1tenth_ws/src/pure_pursuit/waypoints/race2/wp_w_speed_race2_2_9_prak.csv'
        # wp_path = '/home/shubho/ese6150/sim_ws/src/pure_pursuit/waypoints/lev_lobby_lines/interpolated_wp_lev_lobby3.csv'

        # wp_path = '/home/shubho/ese6150/sim_ws/src/pure_pursuit/waypoints/lev_lobby_lines/interpolated_wp_lev_lobby_line2.csv'
        # wp_path = '/home/nvidia/f1tenth_ws/src/pure_pursuit/waypoints/interpolated_wp_levine_obs.csv'
        wp_path = '/home/nvidia/f1tenth_ws/src/pure_pursuit/waypoints/interpolated_wp_levine_2nd.csv'
        # lev_lobby_lines/interpolated_wp_lev_lobby_line2.csv'


        
        self.trajectory = np.array(pd.read_csv(wp_path))
        self.timer = self.create_timer(0.9, self.visualizer_callback)

    def visualizer_callback(self):
        self.visualize(self.trajectory[::10, :2], color=(0.0, 0.0, 1.0), duration=1)

    def visualize(self, points, color=(0.0, 1.0, 0.0), duration=0):
        """Visualize points

        Args:
            points: array-like shape (n, 2)
            color: tuple-like (R, G, B) 
        """
        marker = Marker()
        marker.header.frame_id = "map"  # or "odom", "base_link", etc.
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns, marker.id, marker.type, marker.action = "point", 0, Marker.SPHERE_LIST, Marker.ADD

        # Scale of the sphere (in meters)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05

        # setup color and duration
        marker.color.r, marker.color.g, marker.color.b = color[0], color[1], color[2]
        marker.color.a = 1.0  
        marker.lifetime = Duration(sec=duration)
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in points]
        
        self.waypoints_visualizer.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    print("Waypoints Visualizer Initialized")
    pure_pursuit_node = WaypointVisualizer()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
