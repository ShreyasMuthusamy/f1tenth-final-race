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

eps = 1e-5

class LaneSwitching(Node):
    """ 
    Implement obstacle detection on the car
    """

    def __init__(self):
        super().__init__('lane_switching_node')
        self.config = load_parameters(config_path)

        if self.config['is_sim']:
            odom_topic = '/ego_racecar/odom'
        else:
            odom_topic = '/pf/pose/odom'
        scan_topic = '/scan'
        traj_topic = '/trajectory'

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        # Define subscribers
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.pose_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos)

        # Define publisher
        self.traj_pub = self.create_publisher(Float32MultiArray, traj_topic, qos)

        # load trajectory
        self.trajectories = []
        for wp_name in self.config['controller']['trajectories'].values():
            wp_path = os.path.join(base_dir, f'waypoints/{wp_name}')
            wp_path = os.path.abspath(wp_path)
            self.trajectories.append(np.array(pd.read_csv(wp_path, sep=';')))
        self.curr_traj = self.trajectories[0]

        # Class variables
        self.vehicle_pose = None
        self.speed = None
        self.prev_ranges = None
        self.prev_time = None

    def pose_callback(self, pose_msg: Odometry):
        self.vehicle_pose = self.locate_vehicle(pose_msg)
        self.speed = self.vehicle_pose[3]
    
    def scan_callback(self, scan_msg: LaserScan):
        if self.speed is None:
            return

        ranges, angles = self.preprocess_lidar(scan_msg)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.vstack([xs, ys]).T

        if self.prev_ranges is not None:
            delta_time = (self.get_clock().now() - self.prev_time).nanoseconds * 1e-9
            delta_ranges = self.prev_ranges - ranges
            obstacle_velocity = delta_ranges / delta_time  # only approximated
            projected_speeds = self.speed * np.cos(angles)

            relative_speed = projected_speeds - obstacle_velocity
            relative_speed = np.maximum(relative_speed, eps)  # avoid division by 0
            ttc = ranges / relative_speed
        else:
            relative_speed = np.maximum(self.speed * np.cos(angles), eps)
            ttc = ranges / relative_speed
        
        self.prev_time = self.get_clock().now()

        ttc_threshold = self.config['obstacle_detection']['ttc_threshold']
        mask = ttc < ttc_threshold

        obs = points[mask]

        if len(obs) and self.vehicle_pose is not None:
            x_c, y_c, yaw, _ = self.vehicle_pose
            R = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw), np.cos(yaw)]])
            
            obs = np.einsum('ij,...j->...i', R, obs) + np.array([x_c, y_c])
        
        if len(obs):
            self.curr_traj = self.get_trajectory(obs)
        traj_msg = to_multiarray(self.curr_traj)
        self.traj_pub.publish(traj_msg)
    
    def get_trajectory(self, obs: np.ndarray):
        clearance = self.config['obstacle_detection']['clearance']
        # opt_traj = self.trajectories[0]

        # if not np.array_equal(opt_traj, self.trajectory):
        #     dist = np.linalg.norm(opt_traj[None, :, :2] - obstacles[:, None], axis=-1)
        #     min_dists_opt = np.min(dist, axis=0)

        #     if np.all(min_dists_opt > clearance):
        #         # print('Lane switch! (to optimal)')
        #         self.trajectory = opt_traj
        #         return

        dist = np.linalg.norm(self.curr_traj[None, :, :2] - obs[:, None], axis=-1)
        min_dists_curr = np.min(dist, axis=0)
        if np.all(min_dists_curr > clearance):
            # print('Lane switch! (to current)')
            return self.curr_traj

        for traj in self.trajectories:
            # if np.array_equal(traj, opt_traj) or np.array_equal(traj, self.trajectory):
            if np.array_equal(traj, self.curr_traj):
                continue

            dist = np.linalg.norm(traj[None, :, :2] - obs[:, None], axis=-1)
            min_dists = np.min(dist, axis=0)

            if np.all(min_dists > clearance):
                print('Lane switch!')
                return traj
        
        print('Too fat!')
        return self.curr_traj

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

        ranges = scan_msg.ranges
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        if len(ranges) != len(angles):
            angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Moving average filter - sensor noise is filtered, smoothing filter here
        conv_size = self.config['obstacle_detection']['preprocess_conv_size']
        proc_ranges = np.convolve(ranges, np.ones(conv_size), 'same') / conv_size

        # this helps with nans and inf values
        proc_ranges = np.clip(proc_ranges, 0.0, 6.0)

        return proc_ranges, angles


def main(args=None):
    rclpy.init(args=args)
    print("Lane Swtiching Initialized")
    lane_switcher = LaneSwitching()
    rclpy.spin(lane_switcher)

    lane_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
