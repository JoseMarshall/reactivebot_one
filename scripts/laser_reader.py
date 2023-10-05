import rclpy
from sensor_msgs.msg import LaserScan

def lidar_callback(msg):
    # 720 / 5 = 144
    regions = [
        min(min(msg.ranges[0:143]), 10),
        min(min(msg.ranges[144:287]),10),
        min(min(msg.ranges[288:431]),10),
        min(min(msg.ranges[432:575]),10),
        min(min(msg.ranges[576:713]),10),
    ]
    # Process LiDAR data here
    print("Received LiDAR data")
    print(regions)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('lidar_reader_node')

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
