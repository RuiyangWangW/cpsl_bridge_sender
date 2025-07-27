import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import socket
import struct
import threading
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class WaypointsSender(Node):
    def __init__(self):
        super().__init__('waypoints_tcp_sender')

        self.robot_id_to_ip = {
            'cpsl_uav_1': '10.197.117.67',
            'cpsl_robot_dog_1': '10.197.10.145',
        }

        self.port = 9004  # Different from MapSender port
        self.topic_suffix = '/planned_waypoints'

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sockets = {}
        self.lock = threading.Lock()

        # Subscribe to each robot's waypoints topic
        self.subscribers = []
        for robot_id, ip in self.robot_id_to_ip.items():
            topic = f'/{robot_id}{self.topic_suffix}'
            self.subscribers.append(
                self.create_subscription(PoseArray, topic, self.make_callback(robot_id), qos_profile)
            )
            self.connect_to_receiver(ip)

    def connect_to_receiver(self, ip):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ip, self.port))
            self.sockets[ip] = sock
            self.get_logger().info(f"Connected to receiver at {ip}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {ip}:{self.port} - {e}")

    def make_callback(self, robot_id):
        def callback(msg):
            ip = self.robot_id_to_ip.get(robot_id)
            if ip not in self.sockets:
                self.get_logger().warn(f"No socket connection to {ip} for {robot_id}")
                return

            try:
                msg_bytes = serialize_message(msg)
                topic_bytes = f'/{robot_id}{self.topic_suffix}'.encode('utf-8')

                packet = (
                    struct.pack('>I', len(topic_bytes)) + topic_bytes +
                    struct.pack('>I', len(msg_bytes)) + msg_bytes
                )

                with self.lock:
                    try:
                        self.sockets[ip].sendall(packet)
                    except Exception as e:
                        self.get_logger().warn(f"Disconnected from {ip}, error: {e}")
                        self.sockets[ip].close()
                        del self.sockets[ip]
                        self.connect_to_receiver(ip)
            except Exception as e:
                self.get_logger().error(f"Failed to send waypoints for {robot_id}: {e}")

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsSender()
    try:
        rclpy.spin(node)
    finally:
        with node.lock:
            for sock in node.sockets.values():
                sock.close()
        node.destroy_node()
        rclpy.shutdown()
