import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

# Jetson GPIO may not exist on dev machines; fail gracefully
try:
    import Jetson.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

class EncodersNode(Node):
    def __init__(self):
        super().__init__('encoders_node')

        # Parameters
        self.declare_parameter('left_pin', 11)
        self.declare_parameter('right_pin', 13)
        self.declare_parameter('wheel_radius', 0.05)      # meters
        self.declare_parameter('wheel_base', 0.20)        # meters
        self.declare_parameter('ticks_per_rev', 20)       # ticks per wheel rev
        self.declare_parameter('publish_rate_hz', 20.0)   # odom publish rate
        self.declare_parameter('debounce_ms', 1)          # GPIO debounce

        self.left_pin = int(self.get_parameter('left_pin').value)
        self.right_pin = int(self.get_parameter('right_pin').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.tpr = float(self.get_parameter('ticks_per_rev').value)
        self.debounce_ms = int(self.get_parameter('debounce_ms').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.publisher = self.create_publisher(Odometry, '/encoder_odom', qos)

        self.left_count = 0
        self.right_count = 0
        self.prev_time = time.time()

        if HAS_GPIO:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(self.left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.left_pin, GPIO.RISING,
                                  callback=self.left_callback, bouncetime=self.debounce_ms)
            GPIO.add_event_detect(self.right_pin, GPIO.RISING,
                                  callback=self.right_callback, bouncetime=self.debounce_ms)
        else:
            self.get_logger().warn('Jetson GPIO not available; publishing zeros.')

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_odom)

    def left_callback(self, channel):
        self.left_count += 1

    def right_callback(self, channel):
        self.right_count += 1

    def publish_odom(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0.0:
            return
        self.prev_time = current_time

        # ticks -> linear velocity for each wheel
        # wheel linear distance per tick = (2*pi*r)/tpr
        dist_per_tick = (2.0 * math.pi * self.r) / self.tpr
        left_vel = (self.left_count * dist_per_tick) / dt
        right_vel = (self.right_count * dist_per_tick) / dt
        self.left_count = 0
        self.right_count = 0

        v = 0.5 * (left_vel + right_vel)
        w = (right_vel - left_vel) / self.L

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)

        # Covariances: simple defaults
        odom.twist.covariance = [
            0.05, 0, 0, 0, 0, 0,
            0, 0.2, 0, 0, 0, 0,
            0, 0, 0.5, 0, 0, 0,
            0, 0, 0, 0.2, 0, 0,
            0, 0, 0, 0, 0.2, 0,
            0, 0, 0, 0, 0, 0.05
        ]
        self.publisher.publish(odom)

    def destroy_node(self):
        try:
            if HAS_GPIO:
                GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncodersNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
