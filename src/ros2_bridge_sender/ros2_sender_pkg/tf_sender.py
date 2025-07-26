import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message
import socket
import struct

class TfSender(Node):
    def __init__(self):
        super().__init__('tf_tcp_sender')
        self.allowed_pairs = {
            ('odom', 'base_link')}
        self.tf_prefix = '/cpsl_robot_dog_1'
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.0.64', 9002))  # Change to PC IP and port

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
            filtered_transforms = [
                t for t in msg.transforms
                if (t.header.frame_id, t.child_frame_id) in self.allowed_pairs
            ]

            if not filtered_transforms:
                return  # Nothing to send

            # Add namespace to frame IDs
            for t in filtered_transforms:
                t.header.frame_id = f"{self.tf_prefix}/{t.header.frame_id}"
                t.child_frame_id = f"{self.tf_prefix}/{t.child_frame_id}"

            filtered_msg = TFMessage(transforms=filtered_transforms)
            data = serialize_message(filtered_msg)
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

