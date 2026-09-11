import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TestBaseOscillatorNode(Node):
    """
    Test node that commands base theta rotation back and forth at a configurable angular velocity (rad/s).
    """
    def __init__(self):
        super().__init__('test_base_oscillator')

        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('switch_period', 3.0)   # seconds between direction flips
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('cmd_vel_topic', '/teleop/cmd_vel')

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.switch_period = float(self.get_parameter('switch_period').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.direction = 1.0
        self.last_switch_time = time.time()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"TestBaseOscillatorNode initialized. "
            f"angular_speed={self.angular_speed:.2f} rad/s, switch_period={self.switch_period:.1f}s, topic='{cmd_vel_topic}'"
        )

    def timer_callback(self):
        now = time.time()
        if now - self.last_switch_time >= self.switch_period:
            self.direction *= -1.0
            self.last_switch_time = now
            self.get_logger().info(
                f"Flipping base rotation direction: {self.direction * self.angular_speed:.2f} rad/s"
            )

        msg = Twist()
        msg.angular.z = self.direction * self.angular_speed
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestBaseOscillatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command on exit
        msg = Twist()
        node.publisher.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
