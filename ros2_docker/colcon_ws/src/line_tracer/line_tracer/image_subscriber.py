from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ImageSubscriber(Node):

    def __init__(self, callback=None):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            msg_type=CompressedImage,
            topic='/camera/rgb/image/compressed',
            callback=self._image_callback,
            qos_profile=10
        )
        self.bridge = CvBridge()
        self.callback = callback

    def _image_callback(self, msg):
        # self.get_logger().info('Image Received')
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
            if self.callback is not None:
                self.callback(frame)
        except Exception as e:
            self.get_logger().error(f'Error image callback: {e}')
