import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class SMCSuperellipseNode(Node):

    # ── Trajectory constants ─────────────────────────────────────────────────
    L     = 1.5                        # half side length (m)
    N     = 8.0                        # sharpness: 2→circle, 8→near-square
    T_LAP = 60.0                       # seconds per full lap
    OMEGA = 2.0 * math.pi / T_LAP

    def __init__(self):
        super().__init__('smc_superellipse_node')

        # --- Publishers / Subscribers (identical to smc_tracking_node) ---
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',     10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.odom_sub   = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # --- Robot state ---
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # --- SMC gains (identical to smc_tracking_node) ---
        self.k_v = 0.5
        self.k_w = 1.5
        self.phi = 0.5

        # --- Timing ---
        self.t          = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Fallback yaw when tangent is degenerate
        self.prev_yaw_d = 0.0

        # --- Control timer: 20 Hz (dt = 0.05) ---
        self.dt    = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'SMCSuperellipseNode started. '
            f'L={self.L} m, N={self.N}, T_LAP={self.T_LAP} s')

    # ------------------------------------------------------------------
    def signed_pow(self, base: float, exp: float) -> float:
        """Sign-preserving power: sign(base) * |base|^exp"""
        return math.copysign(abs(base) ** exp, base)

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ------------------------------------------------------------------
    def get_desired_trajectory(self, t: float):
        """
        Superellipse (Lamé curve) parametric trajectory:

            xd(t) = L * signed_pow(cos(omega*t), 2/N)
            yd(t) = L * signed_pow(sin(omega*t), 2/N)

        Analytical derivatives:
            dxd_dt = -L * (2/N) * omega * signed_pow(cos_t, 2/N - 1) * sin_t
            dyd_dt =  L * (2/N) * omega * signed_pow(sin_t, 2/N - 1) * cos_t

        Edge-case guard: exponent (2/N - 1) = -0.75 for N=8 blows up when
        cos_t or sin_t ≈ 0. Clamp those terms to 0.0 and fall back to
        prev_yaw_d if both derivatives vanish simultaneously.
        """
        cos_t = math.cos(self.OMEGA * t)
        sin_t = math.sin(self.OMEGA * t)
        k     = 2.0 / self.N           # shared exponent factor

        # Position
        xd = self.L * self.signed_pow(cos_t, k)
        yd = self.L * self.signed_pow(sin_t, k)

        # Analytical derivatives with edge-case guards
        if abs(cos_t) < 1e-6:
            dxd_dt = 0.0
        else:
            dxd_dt = -self.L * k * self.OMEGA * self.signed_pow(cos_t, k - 1) * sin_t

        if abs(sin_t) < 1e-6:
            dyd_dt = 0.0
        else:
            dyd_dt =  self.L * k * self.OMEGA * self.signed_pow(sin_t, k - 1) * cos_t

        # Desired yaw from tangent direction; keep prev_yaw_d if degenerate
        if dxd_dt == 0.0 and dyd_dt == 0.0:
            yaw_d = self.prev_yaw_d
        else:
            yaw_d = math.atan2(dyd_dt, dxd_dt)
            self.prev_yaw_d = yaw_d

        return xd, yd, yaw_d

    # ------------------------------------------------------------------
    def control_loop(self):
        # Update elapsed time
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.t       = current_time - self.start_time

        # 1. Get desired state from superellipse generator
        xd, yd, yaw_d = self.get_desired_trajectory(self.t)

        # 2. Global-frame errors
        ex    = xd - self.x
        ey    = yd - self.y
        e_yaw = math.atan2(math.sin(yaw_d - self.yaw),
                           math.cos(yaw_d - self.yaw))

        # 3. Rotate errors to body frame
        ex_b =  math.cos(self.yaw) * ex + math.sin(self.yaw) * ey
        ey_b = -math.sin(self.yaw) * ex + math.cos(self.yaw) * ey

        # 4. SMC control law: u = K * tanh(s / phi)
        v_cmd_x = self.k_v * math.tanh(ex_b  / self.phi)
        v_cmd_y = self.k_v * math.tanh(ey_b  / self.phi)
        w_cmd_z = self.k_w * math.tanh(e_yaw / self.phi)

        # 5. Publish velocity command
        twist = Twist()
        twist.linear.x  = v_cmd_x
        twist.linear.y  = v_cmd_y
        twist.angular.z = w_cmd_z
        self.cmd_pub.publish(twist)

        # 6. Publish setpoint for visualiser / logger
        target_msg = PoseStamped()
        target_msg.header.stamp    = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'odom'
        target_msg.pose.position.x = xd
        target_msg.pose.position.y = yd
        target_msg.pose.position.z = 0.0
        self.target_pub.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SMCSuperellipseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
