import rclpy

from cv_bridge import CvBridge

from line_tracer.image_subscriber import ImageSubscriber

"""
ros2 run line_tracer main
"""


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


if __name__ == '__main__':
    main()
