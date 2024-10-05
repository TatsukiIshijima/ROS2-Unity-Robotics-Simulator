import cv2
import rclpy

from line_tracer.line_tracer import LineTracer
from line_tracer.image_subscriber import ImageSubscriber

"""
ros2 run line_tracer line_tracer
"""


def process_image(frame):
    line_tracker = LineTracer()
    frame, moment_cx, moment_cy = line_tracker.process(frame)
    cv2.imshow('LineTracer', frame)
    cv2.waitKey(1)


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
