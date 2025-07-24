import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message
import socket
import struct

class TfSender(Node):
    def __init__(self):
        super().__init__('tf_tcp_sender')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('10.197.79.58', 9002))  # Change to PC IP and port

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)

        self.static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            10)

    def tf_callback(self, msg):
        try:
            data = serialize_message(msg)
            header = struct.pack('>I', len(data))
            self.sock.sendall(header + data)
        except Exception as e:
            self.get_logger().error(f"TF send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

