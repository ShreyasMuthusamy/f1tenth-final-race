#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import yaml
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import os
from std_msgs.msg import Int32

base_dir = '/root/race3/mppi_race3/final_race/'
config_path = os.path.join(base_dir, 'config/race_params.yaml')
config_path = os.path.abspath(config_path)


def load_parameters(file_path):
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


class ReactiveFollowGap(Node):
    BUBBLE_RADIUS            = 40             #how many indices do we want to clear
    PREPROCESS_CONV_SIZE     = 5
    BEST_POINT_CONV_SIZE     = 80
    MAX_LIDAR_DIST           = 3 
    STRAIGHTS_SPEED          = 6.0
    CORNERS_SPEED            = 3.0
    STRAIGHTS_STEERING_ANGLE = 0.15708   # 10 degrees

    def __init__(self):

        super().__init__('reactive_node')

        self.config = load_parameters(config_path)
        lidarscan_topic = '/scan'
        reactive_topic = '/reactive'

        self.subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, reactive_topic, 10)
        self.mask_subscriber = self.create_subscription(Int32, '/mask', self.mask_callback, 10)
        self.radians_per_elem = None

        self.get_logger().info("Naive GapFollow Activated!!!")

        self.mask = 0.0


    def mask_callback(self, msg):
        self.mask = msg.data

    def preprocess_lidar(self, ranges, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        crop_indices = 125

        #angular resolution of the lidar
        self.radians_per_elem = data.angle_increment

        # we won't use the LiDAR data from directly behind us
        # trying to take the forward cone values
        proc_ranges = np.array(ranges[crop_indices:-crop_indices]) #changing this- changes a lot in the sim

        # Moving average filter - sensor noise is filtered, smoothing filter here
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE

        # this helps with nans and inf values
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)

        if self.mask == 1:
            self.get_logger().info("Mask active!!")
            proc_ranges[:len(proc_ranges) // 2] = 0

        return proc_ranges
    

    # input is an array like:  [1, 2, 3, 0, 0, 0, 0, 5, 6, 7, 8, 10, 29]

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """

        # mask the bubble- [1 2 3 -- -- -- -- 5 6 7 8 10 29]
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)

        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)

        # max_length will not work, i want upto 5 or 6 values only 

        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl

            
        # for slice in slices[1:]:


        return chosen_slice.start, chosen_slice.stop



    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        #can be changed- doesnt work if changed

        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i


    #this is wrong ig?? - works on an empty map though
    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle


    def lidar_callback(self, data):
        

        ranges = data.ranges

        # Preprocess the Lidar Information
        proc_ranges = self.preprocess_lidar(ranges, data)

        # Find index of closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)- this can be modified to improve 
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        # preventing out of bounds error
        if min_index < 0: 
            min_index = 0
        if max_index >= len(proc_ranges): 
            max_index = len(proc_ranges) - 1

        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)


        # Find the best point in the gap - yes the largest 
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Get the final steering angle and speed value--?????
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED


        # # Check lidar range from the left and right
        # left_angle_start = int((70 * np.pi / 180) / self.radians_per_elem)
        # left_angle_end = int((85 * np.pi / 180) / self.radians_per_elem)
        # right_angle_start = int((-85 * np.pi / 180) / self.radians_per_elem)
        # right_angle_end = int((-70 * np.pi / 180) / self.radians_per_elem)

        # left_range = np.mean(proc_ranges[len(proc_ranges)//2 + left_angle_start:len(proc_ranges)//2 + left_angle_end])
        # right_range = np.mean(proc_ranges[len(proc_ranges)//2 + right_angle_start:len(proc_ranges)//2 + right_angle_end])

        # # Limit steering angle if range is too small
        # side_threshold = 0.5  # Threshold distance to the side
        # if left_range < side_threshold and steering_angle > 0:
        #     self.get_logger().info("Left side too close! Limiting steering angle.")
        #     steering_angle = min(steering_angle, np.radians(-10))
        # if right_range < side_threshold and steering_angle < 0:
        #     self.get_logger().info("Right side too close! Limiting steering angle.")
        #     steering_angle = max(steering_angle, np.radians(10))


        # # detect obs
        # ranges, angles = self.preprocess_lidar_for_obs(data)

        # xs = ranges * np.cos(angles)
        # ys = ranges * np.sin(angles)
        # points = np.vstack([xs, ys]).T

        # obs = self.detect_obstacles(points)


        # ################ Config for detect obs #########################################################################################################
        # # Check for obstacles in a small cone in front of the car
        # cone_angle = np.radians(30)  # Define the cone angle in radians
        # cone_threshold = 3.0  # Define the threshold distance

        # # Calculate the indices for the cone
        # center_index = len(proc_ranges) // 2
        # cone_width = int(cone_angle / self.radians_per_elem)
        # cone_start = max(center_index - cone_width, 0)
        # cone_end = min(center_index + cone_width, len(proc_ranges) - 1)

        # # Check if any range in the cone is below the threshold

        # if obs.size > 0:
        #     if np.any(proc_ranges[cone_start:cone_end] < cone_threshold) :
        #         self.get_logger().info("Obstacle detected in front cone! Slowing down.")
        #         speed = 3.0  # Slow down if an obstacle is detected
        #         reactive_message = np.array([1.0, speed, steering_angle]) # 1.0  means in the reactive_mode
        #     else:
        #         reactive_message = np.array([0.0, speed, steering_angle])    
        # else:
        #     reactive_message = np.array([0.0, speed, steering_angle])

    
        reactive_message = np.array([0.0, speed, steering_angle])
        reactive_message = to_multiarray(reactive_message)
        self.publisher.publish(reactive_message)

        # #Publish Drive message, speed and steering anlhr
        # drive_msg = AckermannDriveStamped()
        # drive_msg.drive.steering_angle = steering_angle 
        # drive_msg.drive.speed = speed

        # self.publisher.publish(drive_msg)
    
    def detect_obstacles(self, points: np.ndarray) -> np.ndarray:
        """
        Get the locations of the disparities from the point information
        
        Args:
            points (np.ndarray): Point data.
        
        Returns:
            out (np.ndarray): Points at which there are obstacles.
        """
        # lookahead = self.config['obstacle_detection']['lookahead']
        # distance_threshold = self.config['obstacle_detection']['distance_threshold']
        lookahead = 1.5
        distance_threshold = 0.5

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
    
    def preprocess_lidar_for_obs(self, scan_msg: LaserScan) -> np.ndarray:
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

        cone_radius = 200
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
    


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    