from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=1
        )

    def publish(self, twist: Twist):
        self.publisher.publish(twist)
