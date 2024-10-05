from geometry_msgs.msg import Twist
from rclpy.node import Node

from line_tracer.singleton import Singleton

class CmdVelPublisher(Singleton, Node):

    def __init__(self):
        if not hasattr(self, '_initialized'):
            super().__init__('cmd_vel_publisher')
            self.publisher = self.create_publisher(
                msg_type=Twist,
                topic='cmd_vel',
                qos_profile=1
            )
            self._initialized = True

    def publish(self, twist: Twist):
        self.publisher.publish(twist)