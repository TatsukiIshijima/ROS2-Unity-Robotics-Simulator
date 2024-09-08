import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/rgb/image/compressed',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Image Received')
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        cv2.imshow('RgbCameraImage', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()