import math
from shutil import posix

import rclpy
from geometry_msgs.msg import Twist

from line_tracer.cmd_vel_publisher import CmdVelPublisher
from line_tracer.image_subscriber import ImageSubscriber
from line_tracer.line_tracer import LineTracer
from line_tracer.processed_image_publisher import ProcessedImagePublisher

"""
ros2 run line_tracer line_tracer
"""


def process_image(frame):
    line_tracker = LineTracer()
    processed_image_publisher = ProcessedImagePublisher()
    cmd_vel_publisher = CmdVelPublisher()

    frame, moment_cx, moment_cy = line_tracker.process(frame)
    processed_image_publisher.publish(frame)

    twist = Twist()

    if moment_cx is None or moment_cy is None:
        twist.angular.z = 0.0
    else:
        half_width = frame.shape[1] / 2.0
        pos_x_rate = (half_width - moment_cx) / half_width
        twist.angular.z = pos_x_rate * 0.25 * math.pi
        print(f'twist.angular.z: {twist.angular.z}')

    twist.linear.x = 0.05
    cmd_vel_publisher.publish(twist)


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
