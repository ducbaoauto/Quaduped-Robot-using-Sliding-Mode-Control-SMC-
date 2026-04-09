import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


# Lemniscate of Bernoulli parameters
A     = 3.0      # amplitude (m) — half-width of each lobe
OMEGA = 0.06667  # angular speed (rad/s) — same as circular node (~94 s period)


class SMCFigure8Node(Node):
    def __init__(self):
        super().__init__('smc_figure8_node')

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
        self.dt         = 0.05   # 20 Hz, same as circular node
        self.start_time = None   # initialised on first timer tick
        self.timer      = self.create_timer(self.dt, self.control_loop)

        period = (2.0 * math.pi) / OMEGA
        self.get_logger().info(
            f'SMCFigure8Node started. a={A} m, omega={OMEGA} rad/s, '
            f'period={period:.1f} s')

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ------------------------------------------------------------------
    def get_desired_trajectory(self, t: float):
        """
        Lemniscate of Bernoulli (figure-8):

            xd(t) = a * sin(omega * t)
            yd(t) = (a/2) * sin(2 * omega * t)
                  = a * sin(omega*t) * cos(omega*t)

        Tangent (for yaw_d):
            dxd/dt = a * omega * cos(omega * t)
            dyd/dt = a * omega * cos(2 * omega * t)

        yaw_d = atan2(dyd/dt, dxd/dt)
        """
        ot = OMEGA * t

        xd = A * math.sin(ot)
        yd = (A / 2.0) * math.sin(2.0 * ot)

        dxd = A * OMEGA * math.cos(ot)
        dyd = A * OMEGA * math.cos(2.0 * ot)

        yaw_d = math.atan2(dyd, dxd)

        return xd, yd, yaw_d

    # ------------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time

        # 1. Get desired state
        xd, yd, yaw_d = self.get_desired_trajectory(t)

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
    node = SMCFigure8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
