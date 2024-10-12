import rclpy
import math

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from line_tracer.cmd_vel_publisher import CmdVelPublisher
from line_tracer.line_tracer import LineTracer
from line_tracer.processed_image_publisher import ProcessedImagePublisher


class ImageSubscriber(Node):

    def __init__(self, cv_bridge):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            msg_type=CompressedImage,
            topic='/camera/rgb/image/compressed',
            callback=self._image_callback,
            qos_profile=10
        )
        self.bridge = cv_bridge
        self.line_tracer = LineTracer()
        self.processed_image_publisher = ProcessedImagePublisher(cv_bridge=cv_bridge)
        self.cmd_vel_publisher = CmdVelPublisher()

    def _image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
            self._line_trace(frame)
        except Exception as e:
            self.get_logger().error(f'Error image callback: {e}')

    def _line_trace(self, frame):
        processed_image, moment_cx, moment_cy = self.line_tracer.process(frame)
        self.processed_image_publisher.publish(processed_image)

        twist = Twist()

        if moment_cx is None or moment_cy is None:
            twist.angular.z = 0.0
        else:
            half_width = frame.shape[1] / 2.0
            pos_x_rate = (half_width - moment_cx) / half_width
            # 座標系の関係で反転するため-1をかけている
            twist.angular.z = -1.0 * pos_x_rate * 0.05 * math.pi

        twist.linear.x = 0.05
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    cv_bridge = CvBridge()
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber(cv_bridge=cv_bridge)
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
