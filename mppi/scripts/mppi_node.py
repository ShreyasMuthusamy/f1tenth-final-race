#!/usr/bin/env python3
import time, os, sys
import numpy as np
import jax
import jax.numpy as jnp

import rclpy
from rclpy.node import Node
import tf_transformations
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from utils.ros_np_multiarray import to_multiarray_f32, to_numpy_f32

from infer_env import InferEnv
from mppi_tracking import MPPI
import utils.utils as utils
from utils.jax_utils import numpify
import utils.jax_utils as jax_utils
from utils.Track import Track
from pure_pursuit_node import PurePursuit
import time as time
# The following line is commented out as it is not currently in use: #########################
# jax.config.update("jax_compilation_cache_dir", "/home/nvidia/jax_cache") 
import yaml
import os
base_dir = os.path.dirname(os.path.realpath(__file__))
config_path = os.path.join(base_dir, 'config.yaml')
config_path = os.path.abspath(config_path)

map_dir = os.path.join(base_dir, 'waypoints')
map_dir = os.path.abspath(map_dir)
map_path = os.path.join(base_dir, 'waypoints', 'map_info.txt')
map_path = os.path.abspath(map_path)

with open(config_path, 'r') as file:
    config = yaml.safe_load(file)

## This is a demosntration of how to use the MPPI planner with the Roboracer
## Zirui Zang 2025/04/07

class MPPI_Node(Node):
    def __init__(self):
        super().__init__('lmppi_node')
        self.config = utils.ConfigYAML()
        # self.config.load_file('./config.yaml')
        self.config.load_file(config_path)
        self.config.norm_params = np.array(self.config.norm_params).T
        if self.config.random_seed is None:
            self.config.random_seed = np.random.randint(0, 1e6)
        jrng = jax_utils.oneLineJaxRNG(self.config.random_seed)    
        
        # map_info = np.genfromtxt(self.config.map_dir + 'map_info.txt', delimiter='|', dtype='str')
        map_info = np.genfromtxt(map_path, delimiter='|', dtype=str)
        # track, self.config = Track.load_map(self.config.map_dir, map_info, self.config.map_ind, self.config)
        track, self.config = Track.load_map(map_dir, map_info, self.config.map_ind, self.config)
        # track.waypoints[:, 3] += 0.5 * np.pi
        self.infer_env = InferEnv(track, self.config, DT=self.config.sim_time_step)
        self.mppi = MPPI(self.config, self.infer_env, jrng)

        # Do a dummy call on the MPPI to initialize the variables
        state_c_0 = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.control = np.asarray([0.0, 0.0])
        reference_traj, waypoint_ind = self.infer_env.get_refernece_traj(state_c_0.copy(), self.config.ref_vel, self.config.n_steps)
        self.mppi.update(jnp.asarray(state_c_0), jnp.asarray(reference_traj))
        self.get_logger().info('MPPI initialized')
        
        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        # create subscribers
        if self.config.is_sim:
            self.pose_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.pose_callback, qos)
        else:
            self.pose_sub = self.create_subscription(Odometry, "/pf/pose/odom", self.pose_callback, qos)
        # publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", qos)
        self.reference_pub = self.create_publisher(Float32MultiArray, "/reference_arr", qos)
        self.opt_traj_pub = self.create_publisher(Float32MultiArray, "/opt_traj_arr", qos)


        # pure pursuit node for explosive start
        self.pure_pursuit = PurePursuit(speed=config['pp_vel'], lookahead=2.0)
        self.start_time = time.time()
        self.depart_time = 2.0

        self.pp_active = True # NOTE: this parameter is just to print out what mode is running


    def pose_callback(self, pose_msg):
        """
        Callback function for subscribing to particle filter's inferred pose.
        This funcion saves the current pose of the car and obtain the goal
        waypoint from the pure pursuit module.

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        """
        pose = pose_msg.pose.pose
        twist = pose_msg.twist.twist

        # Beta calculated by the arctan of the lateral velocity and the longitudinal velocity
        beta = np.arctan2(twist.linear.y, twist.linear.x)

        # For demonstration, let’s assume we have these quaternion values
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Convert quaternion to Euler angles
        euler = tf_transformations.euler_from_quaternion(quaternion)

        # Extract the Z-angle (yaw)
        theta = euler[2]  # Yaw is the third element

        state_c_0 = np.asarray([
            pose.position.x,
            pose.position.y,
            self.control[0],
            max(twist.linear.x, self.config.init_vel),
            theta,
            twist.angular.z,
            beta,
        ])
        find_waypoint_vel = max(self.config.ref_vel, twist.linear.x)
        reference_traj, waypoint_ind = self.infer_env.get_refernece_traj(state_c_0.copy(), find_waypoint_vel, self.config.n_steps)

        ## MPPI call
        self.mppi.update(jnp.asarray(state_c_0), jnp.asarray(reference_traj))
        mppi_control = numpify(self.mppi.a_opt[0]) * self.config.norm_params[0, :2]/2
        self.control[0] = float(mppi_control[0]) * self.config.sim_time_step + self.control[0]
        self.control[1] = float(mppi_control[1]) * self.config.sim_time_step + twist.linear.x
        
        if self.reference_pub.get_subscription_count() > 0:
            ref_traj_cpu = numpify(reference_traj)
            arr_msg = to_multiarray_f32(ref_traj_cpu.astype(np.float32))
            self.reference_pub.publish(arr_msg)

        if self.opt_traj_pub.get_subscription_count() > 0:
            opt_traj_cpu = numpify(self.mppi.traj_opt)
            arr_msg = to_multiarray_f32(opt_traj_cpu.astype(np.float32))
            self.opt_traj_pub.publish(arr_msg)

        if twist.linear.x < self.config.init_vel:
            self.control = [0.0, self.config.init_vel * 2]

        if np.isnan(self.control).any() or np.isinf(self.control).any():
            self.control = np.array([0.0, 0.0])
            self.mppi.a_opt = np.zeros_like(self.mppi.a_opt)


        current_speed = np.linalg.norm(np.array([
            pose_msg.twist.twist.linear.x,
            pose_msg.twist.twist.linear.y,
            pose_msg.twist.twist.linear.z,
        ]))
        if current_speed < 0.5:
            self.start_time = time.time()

        pp_trajectory = self.pure_pursuit.trajectory 
        
        pp_vehicle_pose = self.pure_pursuit.locate_vehicle(pose_msg)
        pp_closest_point_idx = self.pure_pursuit.find_closetpoint(pp_trajectory, pp_vehicle_pose)
        # ref_speed = pp_trajectory[pp_closest_point_idx, -1] # NOTE: enable this for speed profiling

        # if time.time() - self.start_time <= self.depart_time:
        if False:
            pp_lookahead = self.pure_pursuit.look_ahead       # fox fix loodahead
    
            pp_target_pose = self.pure_pursuit.find_waypoint(pp_trajectory, pp_vehicle_pose, pp_lookahead)
            self.pure_pursuit.visualize([pp_target_pose[:2]], color=(1.0, 0.0, 0.0), duration=1)

            pp_vehicle_coordinate = self.pure_pursuit.global2vehicle_frame(np.expand_dims(pp_target_pose[:2], axis=0), pp_vehicle_pose)
            pp_vehicle_coordinate = np.squeeze(pp_vehicle_coordinate, axis=0)

            pp_curvature = self.pure_pursuit.calculate_curvature(pp_vehicle_coordinate)
            _, pp_steering_angle = self.pure_pursuit.calculate_driving_msg(pp_curvature)

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = "base_link"
            drive_msg.drive.steering_angle = pp_steering_angle
            drive_msg.drive.speed = 5.0
            self.drive_pub.publish(drive_msg)
            print(f'Pure Pursuit mode {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}')

            self.pp_active = True
        
        else:
            if self.pp_active:  
                print(f'Switched to MPPI mode {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}')
                self.pp_active = False 

            # Publish the control command
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = "base_link"
            drive_msg.drive.steering_angle = self.control[0]
            drive_msg.drive.speed = self.control[1] ###### NOTE: this follow the reference speed in config file
            # drive_msg.drive.speed = ref_speed               # NOTE: enable this for speed profiling
            # drive_msg.drive.speed = 6.0 if abs(self.control[0]) <= np.radians(10) else 5.0         
            curvature = calc_xy_curvature_stats(reference_traj, mode='mean')
            # drive_msg.drive.speed = 2.2
            # drive_msg.drive.speed = max(4.0, 4.0 + 3.0 / (1.0 + curvature * 0.5)) 

            print(f'Curvature: {curvature}, Speed: {drive_msg.drive.speed}')
            # self.get_logger().info(f"Steering Angle: {drive_msg.drive.steering_angle}, Speed: {drive_msg.drive.speed}")
            self.drive_pub.publish(drive_msg)


def calc_xy_curvature_stats(ref_traj, mode='mean'):
    """
    Calculate mean or max curvature from x, y columns of reference trajectory.

    Args:
        ref_traj (np.ndarray or jnp.DeviceArray): shape (N, D), x = [:,0], y = [:,1]
        mode (str): 'mean' or 'max'

    Returns:
        float: mean or max curvature
    """

    x = ref_traj[:, 0]
    y = ref_traj[:, 1]
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    curvature = np.abs(dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
    curvature = np.nan_to_num(curvature)  # in case of division by zero

    if mode == 'mean':
        result = np.mean(curvature)
    elif mode == 'max':
        result = np.max(curvature)
    else:
        raise ValueError("kind must be 'mean' or 'max'")
    return float(result)

def main(args=None):
    rclpy.init(args=args)
    mppi_node = MPPI_Node()
    rclpy.spin(mppi_node)

    mppi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()