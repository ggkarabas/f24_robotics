import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math


LINEAR_VEL = 0.15
TURN_VEL = 0.3
STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05
FOLLOW_DISTANCE = 0.3  # Desired distance from the wall
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90
DOORWAY_THRESHOLD = 1.5  # Adjust this threshold based on your environment
TURN_THRESHOLD = 0.5  # Threshold to detect if the robot is stuck against a wall


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower_node")
        self.scan_cleaned = []
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscriber2 = self.create_subscription(
            Odometry,
            "/odom",
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.cmd = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []

        for reading in scan:
            if reading == float("Inf"):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        self.pose_saved = position

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return

        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # Check if the robot is too close to the front wall and needs to turn
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = TURN_VEL
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info("Turning to avoid front obstacle")
            return
        
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = LINEAR_VEL / 4
            if right_lidar_min > left_lidar_min:
                self.cmd.angular.z = -0.2
            self.publisher_.publish(self.cmd)
            return

        #    if right_lidar_min > DOORWAY_THRESHOLD:
        #        self.cmd.angular.z = -0.3  # Enter doorway if detected

        #    # Move forward if no immediate obstacle is in front
        #    if front_lidar_min > LIDAR_AVOID_DISTANCE:
        #        self.cmd.linear.x = LINEAR_VEL
        #    else:
        #        self.cmd.linear.x = 0.1  # Slow down near obstacles

        # Additional logic to rotate if stuck
        else: # front_lidar_min < :  # and right_lidar_min < TURN_THRESHOLD:
            self.cmd.linear.x = LINEAR_VEL
            self.cmd.angular.z = 0.0
            # Wall-following behavior
            if right_lidar_min < .2:
                self.cmd.linear.x = LINEAR_VEL / 2
                self.cmd.angular.z = 0.2  # Turn left slightly to maintain distance
            elif right_lidar_min > 1.0:
                self.cmd.linear.x = LINEAR_VEL / 2
                self.cmd.angular.z = -0.2  # Turn right slightly to maintain distance
            elif right_lidar_min > 0.5:
                self.cmd.linear.x = LINEAR_VEL / 2
                self.cmd.angular.z = -0.1  # Turn right slightly to maintain distance
            # else:
                # self.cmd.angular.z = 0.0  # Go straight if at desired distance
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info("Rotating to recover from being stuck")
        # else:
            # self.publisher_.publish(self.cmd)
            # self.turtlebot_moving = True

        self.get_logger().info("Distance to front obstacle: %f" % front_lidar_min)
        self.get_logger().info("Distance to right wall: %f" % right_lidar_min)
        self.get_logger().info('Publishing command: "%s"' % self.cmd)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollower()
    rclpy.spin(wall_follower_node)
    wall_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
