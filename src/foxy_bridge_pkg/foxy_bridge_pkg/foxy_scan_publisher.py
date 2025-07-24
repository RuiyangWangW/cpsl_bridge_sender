import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LaserSender(Node):
    def __init__(self):
        super().__init__('laser_tcp_sender')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match publisher
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )            
        self.subscription = self.create_subscription(
            LaserScan,
            '/livox/scan_best_effort',
            self.laser_callback,
            qos_profile)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.0.64', 9001))  # Replace with PC IP and port

    def laser_callback(self, msg):
        try:
            data = serialize_message(msg)
            # Prefix message with 4-byte length header
            self.sock.sendall(struct.pack('>I', len(data)) + data)
        except Exception as e:
            self.get_logger().error(f"Failed to send laser scan: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

