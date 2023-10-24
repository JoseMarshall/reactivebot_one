import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Sensor Regions
regions = {
    'bbright': 0,
    'bright': 0,
    'right': 0,
    'fright': 0,
    'front': 0,
    'back': 0,
    'fleft': 0,
    'left': 0,
    'bleft': 0,    
    'bbleft': 0,   
}

over_distance = 10

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
        # print("find_wall_on_left")
        return twist_msg

    def find_wall_on_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = -self.angular_speed
        # print("find_wall_on_right")
        return twist_msg
    
    def go_back_left(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.linear_speed
        twist_msg.angular.z = -self.angular_speed
        # print("go_back_left")
        return twist_msg

    def go_back_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.linear_speed
        twist_msg.angular.z = self.angular_speed
        # print("go_back_right")
        return twist_msg

    def go_back_straight(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.linear_speed
        # print("go_back_straight")
        return twist_msg

    def turn_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_speed
        # print("turn_left")
        return twist_msg

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -self.angular_speed
        # print("turn_right")
        return twist_msg

    def go_straight_ahead(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        # print("go_straight_ahead")
        return twist_msg
     
    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # print("stop")
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
    
    def should_stop(self):
        global regions
        global over_distance

        # loop all the regions and count how many are not over distance and save which one they are
        count = 0
        not_over_distance = []
        for region in regions.keys():
            if regions[region] < over_distance:
                count += 1
                not_over_distance.append(region)
        # if only one region is not over distance and it is one of these: fright, right, bright, bbright
        # it means that the robot is next to "plane" wall and it should stop
        if count == 1 and (not_over_distance[0] in ['fright', 'right', 'bright', 'bbright']):
            return True
        # elif only two regions are not over distance and they are one of these: (fright and right) xor (right and bright) xor (bright and bbright)
        elif count == 2 and ((not_over_distance[0] in ['fright', 'right'] and not_over_distance[1] in ['fright', 'right']) or (not_over_distance[0] in ['right', 'bright'] and not_over_distance[1] in ['right', 'bright']) or (not_over_distance[0] in ['bright', 'bbright'] and not_over_distance[1] in ['bright', 'bbright'])):
            # If the value of the two regions are at least almost the same
            # it means that the robot is next to "plane" wall and it should stop
            if abs(regions[not_over_distance[0]] - regions[not_over_distance[1]]) < self.error_distance:
                return True
        return False
    
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
        
        regions = {
            'right':  min(min(msg.ranges[0:35]), over_distance),
            'fright': min(min(msg.ranges[36:71]), over_distance),
            'front':  min(min(msg.ranges[72:107]), over_distance),
            'fleft':  min(min(msg.ranges[108:143]), over_distance),
            'left':   min(min(msg.ranges[144:179]), over_distance),
            'bleft':  min(min(msg.ranges[180:215]), over_distance),
            'bbleft': min(min(msg.ranges[216:251]), over_distance),
            'back':   min(min(msg.ranges[252:287]), over_distance),
            'bbright': min(min(msg.ranges[288:323]), over_distance),
            'bright': min(min(msg.ranges[324:359]), over_distance),
        }

        # If robot is on stop region (should stop), stop/park it, otherwise take action (keep moving)
        twist_msg = self.stop() if self.should_stop() else self.take_action()
        self.pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    wall_follower_node = WallFollowerNode()

    while rclpy.ok():
        rclpy.spin_once(wall_follower_node)

    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
