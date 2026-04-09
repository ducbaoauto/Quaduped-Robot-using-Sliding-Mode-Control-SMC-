import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


# ── Trajectory parameters ────────────────────────────────────────────────────
L     = 3.0          # half-side length (m) — keeps corners at ±1.5
T     = 60.0         # lap period (s)
OMEGA = 2.0 * math.pi / T
N     = 6.0          # Lamé exponent: 2 → circle, 6 → nicely rounded square


def signed_pow(base: float, exp: float) -> float:
    """Sign-preserving power: sign(x) * |x|^exp"""
    return math.copysign(abs(base) ** exp, base)


class SMCRoundedSquareNode(Node):
    def __init__(self):
        super().__init__('smc_rounded_square_node')

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

        # Fallback: remember last valid yaw_d to handle degenerate tangent points
        self._last_yaw_d = 0.0

        # --- Control timer: 20 Hz (same as circular node) ---
        self.dt    = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'SMCRoundedSquareNode started. '
            f'L={L} m, T={T} s, n={N} (Lamé curve)')

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ------------------------------------------------------------------
    def get_desired_trajectory(self, t: float):
        """
        Lamé curve (superellipse) parametric trajectory:

            xd(t) = L * sign(cos(ωt)) * |cos(ωt)|^(2/n)
            yd(t) = L * sign(sin(ωt)) * |sin(ωt)|^(2/n)

        Analytical tangent (for yaw_d):
            dxd/dt = L*(2/n) * |cos(ωt)|^(2/n - 1) * (-ω*sin(ωt))
            dyd/dt = L*(2/n) * |sin(ωt)|^(2/n - 1) * ( ω*cos(ωt))

        Edge-case guard: when cos(ωt) ≈ 0 or sin(ωt) ≈ 0 the exponent
        (2/n - 1) = (2/6 - 1) = -2/3 would blow up, so clamp to 0.
        If both derivatives vanish, fall back to the previous valid yaw_d.
        """
        ot    = OMEGA * t
        cos_t = math.cos(ot)
        sin_t = math.sin(ot)
        exp_p = 2.0 / N          # position exponent  (e.g. 1/3 for n=6)
        exp_d = exp_p - 1.0      # derivative exponent (e.g. -2/3 for n=6)

        # Position
        xd = L * signed_pow(cos_t, exp_p)
        yd = L * signed_pow(sin_t, exp_p)

        # Tangent — guard near-zero bases
        if abs(cos_t) < 1e-6:
            dxd = 0.0
        else:
            dxd = L * exp_p * (abs(cos_t) ** exp_d) * (-OMEGA * sin_t)

        if abs(sin_t) < 1e-6:
            dyd = 0.0
        else:
            dyd = L * exp_p * (abs(sin_t) ** exp_d) * (OMEGA * cos_t)

        # Desired yaw from tangent direction; fall back if both are zero
        if abs(dxd) < 1e-9 and abs(dyd) < 1e-9:
            yaw_d = self._last_yaw_d
        else:
            yaw_d = math.atan2(dyd, dxd)
            self._last_yaw_d = yaw_d

        return xd, yd, yaw_d

    # ------------------------------------------------------------------
    def control_loop(self):
        # Update elapsed time
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.t       = current_time - self.start_time

        # 1. Get desired state from Lamé curve
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
    node = SMCRoundedSquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
