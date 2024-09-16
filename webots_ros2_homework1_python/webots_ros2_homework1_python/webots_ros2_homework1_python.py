import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import matplotlib.pyplot as plt
import signal
import sys


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
        
        # Variables to track position and distance
        self.previous_position = None
        self.initial_position = None  # New: Track initial position
        self.total_distance = 0.0
        self.positions = []  # List to store positions (x, y)

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

        if self.initial_position is None:
            # Store the initial position as the starting point
            self.initial_position = position

        # Normalize the position relative to the starting point
        normalized_x = position.x - self.initial_position.x
        normalized_y = position.y - self.initial_position.y

        if self.previous_position is not None:
            # Calculate Euclidean distance between current and previous positions
            dx = normalized_x - self.previous_position[0]
            dy = normalized_y - self.previous_position[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            self.total_distance += distance
        
        # Log the current normalized position (x, y) for plotting later
        self.positions.append((normalized_x, normalized_y))
        
        # Update previous position
        self.previous_position = (normalized_x, normalized_y)
        
        # Log total distance covered
        self.get_logger().info(f"Total distance covered: {self.total_distance:.2f} meters")

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
        else:
            self.cmd.linear.x = LINEAR_VEL
            self.cmd.angular.z = 0.0
            if right_lidar_min < .2:
                self.cmd.linear.x = LINEAR_VEL / 2
                self.cmd.angular.z = 0.2  # Turn left slightly to maintain distance
            elif right_lidar_min > 1.0:
                self.cmd.linear.x = LINEAR_VEL / 2
                self.cmd.angular.z = -0.2  # Turn right slightly to maintain distance
            elif right_lidar_min > 0.5:
                self.cmd.linear.x = LINEAR_VEL / 2
                self.cmd.angular.z = -0.1  # Turn right slightly to maintain distance
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True

        self.get_logger().info("Distance to front obstacle: %f" % front_lidar_min)
        self.get_logger().info("Distance to right wall: %f" % right_lidar_min)
        self.get_logger().info('Publishing command: "%s"' % self.cmd)
        
    def save_positions_to_file(self, filename="robot_path.csv"):
        """Save the robot's path to a CSV file for later plotting."""
        with open(filename, "w") as f:
            for position in self.positions:
                f.write(f"{position[0]},{position[1]}\n")

    def plot_path(self):
        """Plot the robot's path using matplotlib."""
        x_vals = [pos[0] for pos in self.positions]
        y_vals = [pos[1] for pos in self.positions]
        plt.plot(x_vals, y_vals, marker='o')
        plt.title("Robot Path")
        plt.xlabel("X Position (meters)")
        plt.ylabel("Y Position (meters)")
        plt.grid(True)
        plt.show()

# Graceful shutdown handler
def signal_handler(sig, frame):
    print("Ctrl+C detected! Saving data and exiting...")
    if 'wall_follower_node' in globals():
        wall_follower_node.save_positions_to_file()
        wall_follower_node.plot_path()
        wall_follower_node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    global wall_follower_node
    wall_follower_node = WallFollower()
    
    # Register the signal handler to handle Ctrl+C (SIGINT)
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.spin(wall_follower_node)

if __name__ == "__main__":
    main()
