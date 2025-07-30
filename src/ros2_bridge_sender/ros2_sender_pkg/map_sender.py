import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import socket
import struct
import threading
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


class MapSender(Node):
    def __init__(self):
        super().__init__(
            'map_sender',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.topic_name = '/map'
        self.port = 9003
        self.target_ips = self.get_parameter('target_ips').value
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.topic_name,
            self.map_callback,
            qos_profile
        )

        # Dict of IP -> socket
        self.sockets = {}
        self.lock = threading.Lock()

        self.connect_to_all_receivers()

    def connect_to_all_receivers(self):
        for ip in self.target_ips:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ip, self.port))
                self.sockets[ip] = sock
                self.get_logger().info(f"Connected to receiver at {ip}:{self.port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to {ip}:{self.port} - {e}")

    def map_callback(self, msg):
        try:
            msg_bytes = serialize_message(msg)
            topic_bytes = self.topic_name.encode('utf-8')

            packet = (
                struct.pack('>I', len(topic_bytes)) + topic_bytes +
                struct.pack('>I', len(msg_bytes)) + msg_bytes
            )

            with self.lock:
                disconnected = []
                for ip, sock in self.sockets.items():
                    try:
                        sock.sendall(packet)
                    except Exception as e:
                        self.get_logger().warn(f"Disconnected from {ip}, error: {e}")
                        sock.close()
                        disconnected.append(ip)

                # Try to reconnect if needed
                for ip in disconnected:
                    try:
                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.connect((ip, self.port))
                        self.sockets[ip] = sock
                        self.get_logger().info(f"Reconnected to {ip}:{self.port}")
                    except Exception as e:
                        self.get_logger().warn(f"Reconnection to {ip} failed: {e}")
                        self.sockets.pop(ip, None)

        except Exception as e:
            self.get_logger().error(f"Failed to send map message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapSender()
    try:
        rclpy.spin(node)
    finally:
        with node.lock:
            for sock in node.sockets.values():
                sock.close()
        node.destroy_node()
        rclpy.shutdown()
