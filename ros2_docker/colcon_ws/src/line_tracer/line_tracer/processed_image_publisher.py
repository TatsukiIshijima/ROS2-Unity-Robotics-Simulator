from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from line_tracer.singleton import Singleton


class ProcessedImagePublisher(Singleton, Node):

    def __init__(self):
        if not hasattr(self, '_initialized'):
            super().__init__('processed_image_publisher')
            self.publisher = self.create_publisher(
                msg_type=CompressedImage,
                topic='/processed/image/compressed',
                qos_profile=1
            )
            self.bridge = CvBridge()
            self._initialized = True

    def publish(self, img):
        compressed_img = self.bridge.cv2_to_compressed_imgmsg(img, dst_format='jpg')
        self.publisher.publish(compressed_img)
