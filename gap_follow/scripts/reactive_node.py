import rclpy
from rclpy.node import Node

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

        self.get_logger().info('Starting ReactiveFollowGap Node')

    def preprocess_lidar(self, ranges):
        self.get_logger().info('Preprocessing LIDAR data')
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        #TODO: figure out how to take average


        # Reject higher vals
        proc_ranges = [0 if data <= 3 else data for data in ranges]
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        self.get_logger().info('Finding max gap')
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        maxLen, maxStart, maxEnd = 0,0,0
        curLen, curStart = 0, None

        for index, value in enumerate(free_space_ranges):
            if value != 0:
                if curStart is None:
                    curStart = index
                curLen += 1
            
            else:
                if curLen > maxLen:
                    maxLen, maxStart = curLen, curStart
                    maxEnd = index - 1
                
                curLen, curStart = 0, None
        
        # Check again at the very end
        # We still have 1 more possibility
        if curLen > maxLen:
            maxLen, maxStart = curLen, curStart
            maxEnd = len(free_space_ranges) - 1

        return (maxStart, maxEnd)
    
    def find_best_point(self, start_i, end_i, ranges):
        self.get_logger().info('Finding best point')
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        bestPoint = start_i + ranges[start_i:end_i+1].index(
                            max(ranges[start_i:end_i+1])
        )
        return bestPoint

    def lidar_callback(self, data):
        self.get_logger().info('Received LIDAR data')
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        closestPoint = proc_ranges.index(min(proc_ranges))
        
        #Eliminate all points inside 'bubble' (set them to zero) 
        if closestPoint < len(proc_ranges):
            bubbleCenter = proc_ranges[closestPoint]
            left, right = closestPoint + 1, closestPoint - 1

            if left < len(proc_ranges):
                p = proc_ranges[left]
                theta = data.angle_increment
                while np.sqrt(bubbleCenter**2 + p**2 
                              - 2*bubbleCenter*p*np.cos(theta)) < 1:
                    proc_ranges[left] = 0
                    left += 1

                    if left >= len(proc_ranges):
                        break
                    
                    p = proc_ranges[left]
                    theta += data.angle_increment

            if right >= 0:
                p = proc_ranges[right]
                theta = data.angle_increment
                while np.sqrt(bubbleCenter**2 + p**2 
                              - 2*bubbleCenter*p*np.cos(theta)) < 1:
                    proc_ranges[right] = 0
                    right -= 1

                    if right <= 0:
                        break
                    
                    p = proc_ranges[right]
                    theta += data.angle_increment


        #Find max length gap
        maxStart, maxEnd = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        bestPoint = self.find_best_point(maxStart, maxEnd, proc_ranges)

        #Publish Drive message
        midPoint = len(ranges) // 2
        angle = data.angle_increment * (bestPoint - midPoint)

        driveMsg = AckermannDriveStamped()
        driveMsg.drive.steering_angle = angle
        driveMsg.drive.speed = 2.0
        self.cmdDrive_pub.publish(driveMsg)

        


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()