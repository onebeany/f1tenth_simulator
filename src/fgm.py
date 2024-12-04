#!/usr/bin/env python3

# python package
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 2.5
    CORNERS_SPEED_1 = 1.0
    CORNERS_SPEED_2 = 0.5
    STRAIGHTS_STEERING_ANGLE_1 = np.pi / 18  # 10 degrees
    STRAIGHTS_STEERING_ANGLE_2 = np.pi / 9  # 20 degrees

    def __init__(self):
        super().__init__('reactive_follow_gap_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        self.ackermann_data = AckermannDriveStamped()

    def scan_callback(self, msg):
        scan_msg = msg
        self.radians_per_elem = (2 * np.pi) / len(scan_msg.ranges)
        proc_ranges = np.array(scan_msg.ranges[135:-135])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)

        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)
        
        # Publish Drive message
        angle = self.get_angle(best, len(proc_ranges))
        self.get_logger().info(f'Best Point: {best+135}, Angle: {angle}')    
        
        if abs(angle) > self.STRAIGHTS_STEERING_ANGLE_2:
            velocity = self.CORNERS_SPEED_2
        elif abs(angle) > self.STRAIGHTS_STEERING_ANGLE_1:
            velocity = self.CORNERS_SPEED_1
        else:
            velocity = self.STRAIGHTS_SPEED
            
        if min(scan_msg.ranges[480:600]) < 0.3:
            velocity = 0.0

        self.ackermann_data.drive.speed = velocity
        self.ackermann_data.drive.steering_angle = angle
        self.publisher_.publish(self.ackermann_data)

    def find_max_gap(self, free_space_ranges):
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop
    
    def find_best_point(self, start_i, end_i, ranges):
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    reactive_follow_gap_node = ReactiveFollowGap()
    rclpy.spin(reactive_follow_gap_node)
    reactive_follow_gap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
