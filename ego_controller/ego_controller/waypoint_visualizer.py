#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from std_msgs.msg import ColorRGBA
import yaml
import os

base_dir = '/home/shreyasm/f1tenth_ws/src/final_race/'
config_path = os.path.join(base_dir, 'config/race_params.yaml')
config_path = os.path.abspath(config_path)

def load_parameters(file_path):
    with open(file_path, "r") as file:
        params = yaml.safe_load(file)
    return params

class WaypointVisualizer(Node):
    """ 
    Visualize the waypoints
    """
    def __init__(self):
        super().__init__('waypoint_viz_node')
        self.config = load_parameters(config_path)

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        
        self.waypoints_visualizer = self.create_publisher(Marker, '/pp_viz/waypoints', qos)
        
        # load trajectory
        self.trajectories = []
        for wp_name in self.config['controller']['trajectories'].values():
            wp_path = os.path.join(base_dir, f'waypoints/{wp_name}')
            wp_path = os.path.abspath(wp_path)
            self.trajectories.append(np.array(pd.read_csv(wp_path, sep=';')))

        self.markers = None
        self.timer = self.create_timer(2, self.visualizer_callback)

    def visualizer_callback(self):
        # self.visualize(self.trajectory[::10, :2], color=(0.0, 0.0, 1.0), duration=1)
        for traj in self.trajectories:
            self.visualize(traj[:, :2], color=traj[:, -2], duration=0)
        # self.visualize(self.trajectory[:, :2], color=self.trajectory[:, -1], duration=0)

    def visualize(self, points, color=(0.0, 1.0, 0.0), duration=0, cmap='viridis'):
        """Visualize points

        Args:
            points: array-like shape (n, 2)
            color: tuple-like (R, G, B) 
        """
        if self.markers is None:
            markers = Marker()
            markers.header.frame_id = "map"  # or "odom", "base_link", etc.
            markers.header.stamp = self.get_clock().now().to_msg()

            markers.ns, markers.id, markers.type, markers.action = "point", 0, Marker.SPHERE_LIST, Marker.ADD

            # Scale of the sphere (in meters)
            markers.scale.x = markers.scale.y = markers.scale.z = 0.05

            markers.lifetime = Duration(sec=duration)
            markers.points = [Point(x=x, y=y, z=0.0) for x, y in points]


            # setup color and duration
            if len(color) == 3:
                markers.color.r, markers.color.g, markers.color.b = color[0], color[1], color[2]
                markers.color.a = 1.0  
            elif len(color) == len(markers.points):
                norm = mcolors.Normalize(vmin=np.min(color), vmax=np.max(color))
                colormap = plt.get_cmap(cmap)  
                markers.colors = []
                for val in color:
                    rgba = colormap(norm(val))
                    color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=1.0)
                    markers.colors.append(color)
            else:
                raise NotImplemented()
            self.markers = markers
            print('Initialize marker!')
        
        print(f"Publish waypoint markers, duration {duration}!")
        self.waypoints_visualizer.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    print("Waypoints Visualizer Initialized")
    pure_pursuit_node = WaypointVisualizer()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
