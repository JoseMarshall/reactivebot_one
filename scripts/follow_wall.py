import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

regions = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

over_distance = 10

stop_region = {
    'limInfX': -0.025,
    'limSupX': 0.065,
    'limInfY': -2,
    'limSupY': -1.58
}

on_stop_region = False

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'lidar_scan', self.scan_callback, 10)
        self.wall_distance = 0.15  # Desired distance from the wall
        self.error_distance = 0.03    # Error distance
        self.linear_speed = 0.08   # Linear speed
        self.angular_speed = 0.2   # Angular speed

    def find_wall_on_left(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = self.angular_speed
        print("find_wall_on_left")
        return twist_msg

    def find_wall_on_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = -self.angular_speed
        print("find_wall_on_right")
        return twist_msg
    
    def go_back_left(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.linear_speed
        twist_msg.angular.z = -self.angular_speed
        print("go_back_left")
        return twist_msg

    def go_back_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.linear_speed
        twist_msg.angular.z = self.angular_speed
        print("go_back_right")
        return twist_msg

    def go_back_straight(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.linear_speed
        print("go_back_straight")
        return twist_msg

    def turn_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_speed
        print("turn_left")
        return twist_msg

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -self.angular_speed
        print("turn_right")
        return twist_msg

    def go_straight_ahead(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        print("go_straight_ahead")
        return twist_msg
     
    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        print("stop")
        return twist_msg

    def is_too_far_from_the_wall(self):
        global regions
        # if all the regions are greater than the desired distance from the wall
        # We consider only the front, front-right and right regions
        # cause this is the region that the robot will use to follow the wall
        return (regions['front'] > self.wall_distance + self.error_distance) and (regions['fright'] > self.wall_distance + self.error_distance) and (regions['right'] > self.wall_distance + self.error_distance)

    def is_too_close_to_the_wall(self):
        global regions
        # if all the regions are smaller than the desired distance from the wall
        # We consider only the front, front-right and right regions
        # cause this is the region that the robot will use to follow the wall
        return (regions['front'] <= self.wall_distance - self.error_distance) or (regions['fright'] <= self.wall_distance - self.error_distance) or (regions['right'] <= self.wall_distance - self.error_distance)
    
    def take_action(self):
        global regions
        global over_distance

         # If the robot is too close to the wall
        if self.is_too_close_to_the_wall():
            # If left side is closer to the wall than the right side
            if (regions['fleft'] <= regions['fright']) or (regions['left'] <= regions['right']):
                return self.go_back_right()
            else:
                return self.go_back_straight()
        
        #If the robot is pointed to the wall
        elif (regions['front'] < over_distance or regions['fleft'] < over_distance or regions['fright'] < over_distance)and regions['right'] == over_distance and regions['left'] == over_distance:
            return self.go_straight_ahead()

        # If the robot is too far from the wall
        elif self.is_too_far_from_the_wall():
            # If left side is closer to the wall than the right side
            if ((regions['fleft'] < regions['fright']) or (regions['left'] < regions['right'])) and (regions['right'] < over_distance):
                return self.find_wall_on_left()
            else:
                return self.find_wall_on_right()
        
        # If the robot is at the desired distance from the wall
        else:
            # If the front region is clear
            if regions['front'] > self.wall_distance and regions['right'] > self.wall_distance and regions['right'] < self.wall_distance + self.error_distance:
                return self.go_straight_ahead()
            # If the front region is not clear
            else:
                # If the left or front-left region is clear
                if (regions['fleft'] > self.wall_distance) or (regions['left'] > self.wall_distance):
                    return self.turn_left()
                # If the right or front-right region is clear
                elif (regions['fright'] > self.wall_distance) or (regions['right'] > self.wall_distance):
                    return self.turn_right()

    def scan_callback(self, msg: LaserScan):
        global regions
        global over_distance
        global on_stop_region
        
        regions = {
            'right':  min(min(msg.ranges[0:35]), over_distance),
            'fright': min(min(msg.ranges[36:71]), over_distance),
            'front':  min(min(msg.ranges[72:107]), over_distance),
            'fleft':  min(min(msg.ranges[108:143]), over_distance),
            'left':   min(min(msg.ranges[144:179]), over_distance),
        }
    
        # If robot is on stop region, stop/park it, otherwise take action (keep moving)
        twist_msg = self.stop() if on_stop_region else self.take_action()
        self.pub.publish(twist_msg)

    def timer_callback(self):
        return

    def odom_callback(self, msg: Odometry):
        global stop_region
        global on_stop_region
        # Extract the position (x, y, z) from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        on_stop_region = x >=stop_region['limInfX'] and x <=stop_region['limSupX'] and y >=stop_region['limInfY'] and y <=stop_region['limSupY']

        # print('Robot position: x={}, y={}'.format(x, y))

def main(args=None):
    rclpy.init(args=args)
    
    # Set the loop rate to achieve a desired frequency (e.g., 20 Hz)
    loop_rate = 40  # Adjust the rate as needed
    timer_period = 1.0 / loop_rate  # Time in seconds between iterations

    wall_follower_node = WallFollowerNode()
    timer = wall_follower_node.create_timer(timer_period, wall_follower_node.timer_callback)

    robot_position_listener = rclpy.create_node('robot_position_listener')
    # Subscribe to the Odometry topic to get the robot's pose
    robot_position_listener.create_subscription(Odometry, '/odom', wall_follower_node.odom_callback, 10)

    while rclpy.ok():
        rclpy.spin_once(wall_follower_node)
        rclpy.spin_once(robot_position_listener)

    wall_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
