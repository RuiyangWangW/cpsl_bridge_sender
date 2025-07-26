import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message
import socket
import struct
from collections import defaultdict


class TfSender(Node):
    def __init__(self):
        super().__init__('tf_tcp_sender')

        # Map of (parent, child) frame pairs -> (IP, port)
        self.pair_to_ip = {
            ('odom', 'base_link'): ('192.168.0.64', 9002),
            ('map', 'base_footprint'): ('192.168.0.101', 9002),
            # Add more pairs as needed
        }

        # Add tf_prefix per receiver if needed (optional)
        self.pair_to_prefix = {
            ('odom', 'base_link'): '/cpsl_robot_dog_1',
            ('map', 'base_footprint'): '/cpsl_uav_1',
        }

        self.sockets = {}
        self.connect_sockets()

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            10
        )

    def connect_sockets(self):
        """Create and store one socket per unique IP."""
        unique_ips = set(self.pair_to_ip.values())
        for ip, port in unique_ips:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ip, port))
                self.sockets[(ip, port)] = sock
                self.get_logger().info(f"Connected to {(ip, port)}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to {(ip, port)}: {e}")

    def tf_callback(self, msg):
        try:
            # Group transforms by destination (ip, port)
            group_by_dest = defaultdict(list)

            for t in msg.transforms:
                key = (t.header.frame_id, t.child_frame_id)
                if key in self.pair_to_ip:
                    dest = self.pair_to_ip[key]
                    prefix = self.pair_to_prefix.get(key, '')
                    # Add prefix
                    t.header.frame_id = f"{prefix}/{t.header.frame_id}"
                    t.child_frame_id = f"{prefix}/{t.child_frame_id}"
                    group_by_dest[dest].append(t)

            for dest, transform_list in group_by_dest.items():
                filtered_msg = TFMessage(transforms=transform_list)
                data = serialize_message(filtered_msg)
                header = struct.pack('>I', len(data))

                sock = self.sockets.get(dest)
                if sock:
                    try:
                        sock.sendall(header + data)
                    except Exception as e:
                        self.get_logger().error(f"Failed to send to {dest}: {e}")
                else:
                    self.get_logger().warn(f"No socket found for {dest}")

        except Exception as e:
            self.get_logger().error(f"TF send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TfSender()
    try:
        rclpy.spin(node)
    finally:
        for sock in node.sockets.values():
            sock.close()
        node.destroy_node()
        rclpy.shutdown()

