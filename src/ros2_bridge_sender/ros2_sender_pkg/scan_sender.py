import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class LaserSender(Node):
    def __init__(self):
        super().__init__(
            'laser_sender',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        # Set the topic you want to send
        self.topic_name = self.get_parameter('topic_name').value
        self.target_ip = self.get_parameter('target_ip').value
        self.robot_id = self.get_parameter('value').value

        self.robot_id = self.get_parameter('robot_id').value              # Used to remap topic

        # Topic to send over TCP
        self.send_topic = f"/{self.robot_id}/scan"
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match receiver QoS if needed
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create the subscriber to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            self.topic_name,
            self.laser_callback,
            qos_profile
        )

        # Connect to the receiver (replace IP if needed)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.target_ip, 9001))  # Replace with receiver IP
        self.get_logger().info(f"Connected to receiver at {self.target_ip}:9001")

    def laser_callback(self, msg):
        try:
            msg_bytes = serialize_message(msg)
            topic_bytes = self.send_topic.encode('utf-8')

            # Format:
            # [topic_len][topic_name][msg_len][serialized msg]
            packet = (
                struct.pack('>I', len(topic_bytes)) + topic_bytes +
                struct.pack('>I', len(msg_bytes)) + msg_bytes
            )

            self.sock.sendall(packet)
        except Exception as e:
            self.get_logger().error(f"Failed to send laser scan: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LaserSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

