import cv2
import rclpy

from line_tracer.line_tracer import LineTracer
from line_tracer.image_subscriber import ImageSubscriber
from line_tracer.processed_image_publisher import ProcessedImagePublisher

"""
ros2 run line_tracer line_tracer
"""


def process_image(frame):
    line_tracker = LineTracer()
    frame, moment_cx, moment_cy = line_tracker.process(frame)
    processed_image_publisher = ProcessedImagePublisher()
    processed_image_publisher.publish(frame)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber(callback=process_image)

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
