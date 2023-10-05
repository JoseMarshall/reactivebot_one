#! /usr/bin/env python
import rclpy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None
linear_x = 0.0
angular_z = 0.0
OBSTACLE_DISATNCE = 0.3

def lidar_callback(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 1),
        'fright': min(min(msg.ranges[144:287]), 1),
        'front':  min(min(msg.ranges[288:431]), 1),
        'fleft':  min(min(msg.ranges[432:575]), 1),
        'left':   min(min(msg.ranges[576:719]), 1),
    }

    take_action(regions)

def turn_left():
    global linear_x 
    global angular_z 
    linear_x = 0.0
    angular_z = -0.3

def turn_right():
    global linear_x 
    global angular_z 
    linear_x = 0.0
    angular_z = 0.3

def go_straight():
    global linear_x 
    global angular_z 
    linear_x = 0.6
    angular_z = 0.0

def take_action(regions):
    msg = Twist()
   
    state_description = ''

    if regions['front'] > OBSTACLE_DISATNCE and regions['fleft'] > OBSTACLE_DISATNCE and regions['fright'] > OBSTACLE_DISATNCE:
        state_description = 'no obstacle'
        go_straight()
    elif regions['front'] < OBSTACLE_DISATNCE and regions['fleft'] > OBSTACLE_DISATNCE and regions['fright'] > OBSTACLE_DISATNCE:
        state_description = 'obstacle on - front'
        turn_left()
    elif regions['front'] > OBSTACLE_DISATNCE and regions['fleft'] > OBSTACLE_DISATNCE and regions['fright'] < OBSTACLE_DISATNCE:
        state_description = 'obstacle on - fright'
        turn_left()
    elif regions['front'] > OBSTACLE_DISATNCE and regions['fleft'] < OBSTACLE_DISATNCE and regions['fright'] > OBSTACLE_DISATNCE:
        state_description = 'obstacle on - fleft'
        turn_right()
    elif regions['front'] < OBSTACLE_DISATNCE and regions['fleft'] > OBSTACLE_DISATNCE and regions['fright'] < OBSTACLE_DISATNCE:
        state_description = 'obstacle on - front and fright'
        turn_left()
    elif regions['front'] < OBSTACLE_DISATNCE and regions['fleft'] < OBSTACLE_DISATNCE and regions['fright'] > OBSTACLE_DISATNCE:
        state_description = 'obstacle on - front and fleft'
        turn_right()
    elif regions['front'] < OBSTACLE_DISATNCE and regions['fleft'] < OBSTACLE_DISATNCE and regions['fright'] < OBSTACLE_DISATNCE:
        state_description = 'obstacle on - front and fleft and fright'
        turn_left()
    elif regions['front'] > OBSTACLE_DISATNCE and regions['fleft'] < OBSTACLE_DISATNCE and regions['fright'] < OBSTACLE_DISATNCE:
        state_description = 'obstacle on - fleft and fright'
        turn_left()
    else:
        state_description = 'unknown case'
    
    print(regions)

    print(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main(args=None):
    global pub
    rclpy.init(args=args)
    node = rclpy.create_node('lidar_reader_node')

    pub = node.create_publisher(
        Twist,
        '/cmd_vel',
        1,
    )

    # Subscribe to the LiDAR topic
    subscription = node.create_subscription(
        LaserScan,
        'lidar_scan',  # Replace with your LiDAR topic
        lidar_callback,
        10  # Adjust the queue size if needed
    )

    rclpy.spin(node=node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()