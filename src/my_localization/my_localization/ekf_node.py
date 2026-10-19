import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_from_quat(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class EkfLocalizer(Node):
    def __init__(self):
        super().__init__('ekf_localizer')
        self.declare_parameter('q_xy', 0.01)    # process noise (position)
        self.declare_parameter('q_yaw', 0.02)   # process noise (heading)
        self.declare_parameter('r_yaw', 0.05)   # IMU yaw noise (std, rad)

        self.x = np.zeros(3)                    # [x, y, yaw]
        self.P = np.eye(3) * 0.1
        self.last_t = None
        self.initialized = False
        self.imu_offset = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu', self.imu_cb, 50)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.initialized:
            p = msg.pose.pose
            self.x = np.array([p.position.x, p.position.y,
                               yaw_from_quat(p.orientation)])
            self.initialized = True
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0:
            return

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        th = self.x[2]

        # PREDICT
        self.x[0] += v * dt * np.cos(th)
        self.x[1] += v * dt * np.sin(th)
        self.x[2] = wrap(th + w * dt)

        F = np.array([[1, 0, -v * dt * np.sin(th)],
                      [0, 1,  v * dt * np.cos(th)],
                      [0, 0, 1]])
        qxy = self.get_parameter('q_xy').value
        qyaw = self.get_parameter('q_yaw').value
        Q = np.diag([qxy, qxy, qyaw]) * dt
        self.P = F @ self.P @ F.T + Q

        self.publish(msg.header.stamp)

    def imu_cb(self, msg):
        if not self.initialized:
            return
        yaw_imu = yaw_from_quat(msg.orientation)
        if self.imu_offset is None:
            # align IMU yaw frame with the odom frame at startup
            self.imu_offset = self.x[2] - yaw_imu
        z = wrap(yaw_imu + self.imu_offset)

        # UPDATE
        H = np.array([[0.0, 0.0, 1.0]])
        r = self.get_parameter('r_yaw').value
        R = np.array([[r * r]])
        y = wrap(z - self.x[2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K * y).flatten()
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

    def publish(self, stamp):
        m = PoseWithCovarianceStamped()
        m.header.stamp = stamp
        m.header.frame_id = 'odom'
        m.pose.pose.position.x = float(self.x[0])
        m.pose.pose.position.y = float(self.x[1])
        m.pose.pose.orientation.z = float(np.sin(self.x[2] / 2))
        m.pose.pose.orientation.w = float(np.cos(self.x[2] / 2))
        cov = np.zeros(36)
        cov[0], cov[1], cov[6], cov[7] = self.P[0, 0], self.P[0, 1], self.P[1, 0], self.P[1, 1]
        cov[35] = self.P[2, 2]
        m.pose.covariance = cov.tolist()
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(EkfLocalizer())
    rclpy.shutdown()
