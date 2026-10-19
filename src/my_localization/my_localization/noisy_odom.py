import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class NoisyOdom(Node):
    def __init__(self):
        super().__init__('noisy_odom')
        self.declare_parameter('v_scale_err', 0.05)  # 5 % wheel scale error
        self.declare_parameter('curv_bias', 0.05)       # rad/s gyro bias
        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.create_subscription(Odometry, '/odom', self.cb, 20)
        self.pub = self.create_publisher(Odometry, '/odom_noisy', 10)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            p = msg.pose.pose
            q = p.orientation
            self.x, self.y = p.position.x, p.position.y
            self.yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0:
            return

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        v_n = v * (1.0 + self.get_parameter('v_scale_err').value) + np.random.randn() * 0.02 * abs(v)
        w_n = w + self.get_parameter('curv_bias').value * v + np.random.randn() * 0.02 * (abs(v) + abs(w))

        self.yaw += w_n * dt
        self.x += v_n * dt * np.cos(self.yaw)
        self.y += v_n * dt * np.sin(self.yaw)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = float(self.x)
        out.pose.pose.position.y = float(self.y)
        out.pose.pose.orientation.z = float(np.sin(self.yaw / 2))
        out.pose.pose.orientation.w = float(np.cos(self.yaw / 2))
        out.twist.twist.linear.x = float(v_n)
        out.twist.twist.angular.z = float(w_n)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(NoisyOdom())
    rclpy.shutdown()