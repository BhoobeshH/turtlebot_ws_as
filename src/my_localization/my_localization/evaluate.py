import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def t_of(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.gt, self.odom, self.ekf = [], [], []
        self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 50)
        self.create_subscription(Odometry, '/odom_noisy', self.odom_cb, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.ekf_cb, 50)

    def gt_cb(self, m):
        p = m.pose.pose.position
        self.gt.append([t_of(m), p.x, p.y])

    def odom_cb(self, m):
        p = m.pose.pose.position
        self.odom.append([t_of(m), p.x, p.y])

    def ekf_cb(self, m):
        p = m.pose.pose.position
        self.ekf.append([t_of(m), p.x, p.y])

    def report(self):
        if min(len(self.gt), len(self.odom), len(self.ekf)) < 2:
            print('Not enough data recorded.')
            return
        # express every trajectory relative to its own first sample
        def rel(a):
            a = np.array(a)
            a[:, 1:] -= a[0, 1:]
            return a
        gt, od, ek = rel(self.gt), rel(self.odom), rel(self.ekf)

        def err(est):
            x = np.interp(gt[:, 0], est[:, 0], est[:, 1])
            y = np.interp(gt[:, 0], est[:, 0], est[:, 2])
            return np.hypot(x - gt[:, 1], y - gt[:, 2])

        for name, est in (('Noisy odometry', od), ('EKF', ek)):
            e = err(est)
            print(f'{name}: RMSE = {np.sqrt(np.mean(e**2)):.3f} m, '
                  f'final error = {e[-1]:.3f} m, max error = {e.max():.3f} m')

        plt.figure(figsize=(6, 6))
        plt.plot(gt[:, 1], gt[:, 2], 'k-', lw=2, label='Ground truth')
        plt.plot(od[:, 1], od[:, 2], 'r--', label='Noisy odometry')
        plt.plot(ek[:, 1], ek[:, 2], 'b-', label='EKF (odometry + IMU)')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Estimated vs ground-truth trajectory')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.savefig('trajectory.png', dpi=200, bbox_inches='tight')
        print('Saved trajectory.png')


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()