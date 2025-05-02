#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from builtin_interfaces.msg import Duration
import pandas as pd
import yaml
import os

base_dir = '/home/shreyasm/f1tenth_ws/src/final_race/'
config_path = os.path.join(base_dir, 'config/race_params.yaml')
config_path = os.path.abspath(config_path)

def load_parameters(file_path: str):
    with open(file_path, "r") as file:
        params = yaml.safe_load(file)
    return params

def to_multiarray(np_array: np.ndarray):
    multiarray = Float32MultiArray()
    multiarray.layout.dim = [MultiArrayDimension(label='dim%d' % i,
                                                 size=np_array.shape[i],
                                                 stride=np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)]
    multiarray.data = np_array.reshape([1, -1])[0].tolist()
    return multiarray

class ObstacleDetection(Node):
    """ 
    Implement obstacle detection on the car
    """

    def __init__(self):
        super().__init__('obstacle_node')
        self.config = load_parameters(config_path)

        if self.config['is_sim']:
            odom_topic = '/ego_racecar/odom'
        else:
            odom_topic = '/pf/pose/odom'
        scan_topic = '/scan'
        obs_topic = '/obstacles'

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        # Define subscribers
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.pose_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos)

        # Define publisher
        self.obs_pub = self.create_publisher(Float32MultiArray, obs_topic, qos)

        self.vehicle_pose = None

    def pose_callback(self, pose_msg: Odometry):
        self.vehicle_pose = self.locate_vehicle(pose_msg)
    
    def scan_callback(self, scan_msg: LaserScan):
        ranges, angles = self.preprocess_lidar(scan_msg)

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.vstack([xs, ys]).T

        obs = self.detect_obstacles(points)

        if len(obs) and self.vehicle_pose is not None:
            x_c, y_c, yaw, _ = self.vehicle_pose
            R = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw), np.cos(yaw)]])
            
            obs = np.einsum('ij,...j->...i', R, obs) + np.array([x_c, y_c])
            obs_msg = to_multiarray(obs)
            self.obs_pub.publish(obs_msg)

        # if self.vehicle_pose is not None:
        #     points_msg = to_multiarray(points)
        #     self.obs_pub.publish(points_msg)

    def locate_vehicle(self, msg: Odometry):
        """Extract vehicle pose from the Odometry message.

        Returns:
            np.array: [x, y, yaw, speed]
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        yaw = euler_from_quaternion(quaternion)[2]
        speed = np.linalg.norm(np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]))

        return np.array([x, y, yaw, speed])
    
    def preprocess_lidar(self, scan_msg: LaserScan) -> np.ndarray:
        """
        Preprocess the LiDAR scan array with the following pipeline:
            1. Clipping the ranges viewed
            2. Setting each value to the mean over some window
            3. Rejecting high values above 3 m
        
        Args:
            ranges (np.ndarray): Ranges to be preprocessed.
        
        Returns:
            out (np.ndarray): Ranges after preprocessing.
        """

        cone_radius = self.config['obstacle_detection']['cone_radius']
        ranges = scan_msg.ranges
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        if len(ranges) != len(angles):
            angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # we won't use the LiDAR data from directly behind us
        # trying to take the forward cone values
        proc_ranges = np.array(ranges[len(ranges)//2-cone_radius:len(ranges)//2+cone_radius])
        angles = angles[len(ranges)//2-cone_radius:len(ranges)//2+cone_radius]

        # Moving average filter - sensor noise is filtered, smoothing filter here
        conv_size = self.config['obstacle_detection']['preprocess_conv_size']
        proc_ranges = np.convolve(proc_ranges, np.ones(conv_size), 'same') / conv_size

        # this helps with nans and inf values
        proc_ranges = np.clip(proc_ranges, 0.0, 5.0)

        return proc_ranges, angles
    
    def detect_obstacles(self, points: np.ndarray) -> np.ndarray:
        """
        Get the locations of the disparities from the point information
        
        Args:
            points (np.ndarray): Point data.
        
        Returns:
            out (np.ndarray): Points at which there are obstacles.
        """
        lookahead = self.config['obstacle_detection']['lookahead']
        distance_threshold = self.config['obstacle_detection']['distance_threshold']

        mask = np.linalg.norm(points, ord=2, axis=1) < lookahead
        obs_idxs = []
        idx_start = -1
        idx_end = -1
        for i in range(1, len(mask)):
            if mask[i] and not mask[i-1]:
                idx_start = i
            if not mask[i] and mask[i-1]:
                idx_end = i
                if np.linalg.norm(points[idx_end] - points[idx_start], ord=2) < distance_threshold:
                    obs_idxs.append(np.arange(idx_start, idx_end))

        return points[np.concatenate(obs_idxs)] if len(obs_idxs) else np.array([])


def main(args=None):
    rclpy.init(args=args)
    print("Obstacle Detection Initialized")
    obstacle_node = ObstacleDetection()
    rclpy.spin(obstacle_node)

    obstacle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
