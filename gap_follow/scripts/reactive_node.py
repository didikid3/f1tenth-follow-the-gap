#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.logging 

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.laserScan_sub = self.create_subscription(LaserScan,
                                                      lidarscan_topic,
                                                      self.lidar_callback,
                                                      10)
        # TODO: Publish to drive
        self.cmdDrive_pub = self.create_publisher(AckermannDriveStamped,
                                                  drive_topic,
                                                  10)
        self.lastProcessedRange = np.zeros((1080,), dtype=np.float32)
        self.lastAngle, self.lastSpeed = None, None
        self.get_logger().info('Starting ReactiveFollowGap Node')

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        #TODO: figure out how to take average
        maxVal = 2
        proc_ranges = np.array(ranges, dtype=np.float32)
        ranges = (proc_ranges + self.lastProcessedRange) / 2
        self.lastProcessedRange = proc_ranges
        np.clip(ranges, 0, maxVal, out=ranges)

        # Reject higher vals
        return ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        split_idx = np.where(free_space_ranges == 0.0)[0]
        sranges = np.split(free_space_ranges,split_idx)
        len_sranges = np.array([len(x) for x in sranges])
        max_idx = np.argmax(len_sranges)
        if max_idx == 0:
            start_i = 0
            end_i = len_sranges[0]-1
        else:
            start_i = np.sum(len_sranges[:max_idx])
            end_i = start_i+len_sranges[max_idx]-1
        max_length_ranges = sranges[max_idx]
        return start_i, end_i, max_length_ranges
        
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        idx_list = np.where(ranges == np.max(ranges))[0]
        best_idx = start_i + idx_list[round(len(idx_list)/2)] 
        return best_idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closestPoint = min(
            range(len(ranges)), key=lambda x: ranges[x]
        )
        
        #Eliminate all points inside 'bubble' (set them to zero) 
        left = max(0, closestPoint - 150)
        right = min(len(proc_ranges) - 1, closestPoint + 149)
        for i in range(left, right + 1):
            proc_ranges[i] = 0

        #Find max length gap
        maxStart, maxEnd, maxRanges = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        bestPoint = self.find_best_point(maxStart, maxEnd, maxRanges)

        #Publish Drive message
        angle = data.angle_increment * bestPoint + data.angle_min

        if self.lastAngle:
            angle, self.lastAngle = angle*0.85 + self.lastAngle*0.15, angle


        # straight
        if 0 < abs(angle) < np.pi/9:
            speed = 4.0

        # Turning into corner
        elif np.pi/9 <= abs(angle) < np.pi/6:
            speed = 2.0

        # correcting
        else:
            speed = 1.0

        if self.lastSpeed:
            speed, self.lastSpeed = speed*0.85 + self.lastSpeed*0.15, speed
        else:
            self.lastSpeed = speed


        driveMsg = AckermannDriveStamped()
        driveMsg.drive.steering_angle = angle
        driveMsg.drive.speed = speed
        self.cmdDrive_pub.publish(driveMsg)
        self.get_logger().info('Speed:{}\tAngle:{}'.format(speed, angle))

        


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()