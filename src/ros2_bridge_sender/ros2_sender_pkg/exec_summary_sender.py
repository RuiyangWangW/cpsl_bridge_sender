import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import struct
import threading
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ExecutionSummarySender(Node):
    def __init__(self):
        super().__init__('execution_summary_tcp_sender')

        self.declare_parameter('robot_id', 'cpsl_uav_1')  # set per robot
        self.robot_id = self.get_parameter('robot_id').value

        self.topic_name = f'/{self.robot_id}/execution_summary'
        self.target_ips = ['10.197.117.67']  # ✅ central planner IP(s)
        self.port = 9005

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            String,
            '/execution_summary',
            self.callback,
            qos_profile
        )

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

    def callback(self, msg):
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
            self.get_logger().error(f"Failed to send execution summary: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ExecutionSummarySender()
    try:
        rclpy.spin(node)
    finally:
        with node.lock:
            for sock in node.sockets.values():
                sock.close()
        node.destroy_node()
        rclpy.shutdown()
