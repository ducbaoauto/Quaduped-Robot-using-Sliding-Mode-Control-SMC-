import math
import csv
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from tf_transformations import euler_from_quaternion


LOG_PATH = '/home/ducbao/go1_ws/smc_data.csv'

CSV_COLUMNS = [
    'timestamp',
    'x_des', 'y_des',
    'x_act', 'y_act', 'yaw_act',
    'sliding_surface_x', 'sliding_surface_y',
    'cmd_vel_linear_x', 'cmd_vel_angular_z',
]


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # --- Latest message cache ---
        self._x_act = 0.0
        self._y_act = 0.0
        self._yaw_act = 0.0

        self._x_des = 0.0
        self._y_des = 0.0

        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0

        self._odom_received = False
        self._target_received = False

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # /target_pose is what smc_tracking_node and smc_square_node both publish
        self.create_subscription(PoseStamped, '/target_pose', self._target_cb, 10)

        # /cmd_vel carries the actual SMC output commands
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # /joint_states — optional, subscribed but not logged to CSV (extend if needed)
        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)

        # --- CSV setup ---
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        self._csv_file = open(LOG_PATH, 'w', newline='')
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._csv_file.flush()

        # --- 50 Hz logging timer ---
        self.create_timer(1.0 / 50.0, self._log_cb)

        self.get_logger().info(f'DataLoggerNode started. Writing to {LOG_PATH}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        self._x_act = msg.pose.pose.position.x
        self._y_act = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self._yaw_act = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._odom_received = True

    def _target_cb(self, msg: PoseStamped):
        self._x_des = msg.pose.position.x
        self._y_des = msg.pose.position.y
        self._target_received = True

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_linear_x = msg.linear.x
        self._cmd_angular_z = msg.angular.z

    def _joint_states_cb(self, msg: JointState):
        # Joint states received — available for future extension
        pass

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log_cb(self):
        if not (self._odom_received and self._target_received):
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Global-frame position error
        ex = self._x_des - self._x_act
        ey = self._y_des - self._y_act

        # Rotate to body frame — mirrors the SMC control law in smc_tracking_node
        # sliding surface s = body-frame error (s_x drives v_x, s_y drives v_y)
        cos_yaw = math.cos(self._yaw_act)
        sin_yaw = math.sin(self._yaw_act)
        sliding_surface_x =  cos_yaw * ex + sin_yaw * ey
        sliding_surface_y = -sin_yaw * ex + cos_yaw * ey

        row = {
            'timestamp':         timestamp,
            'x_des':             self._x_des,
            'y_des':             self._y_des,
            'x_act':             self._x_act,
            'y_act':             self._y_act,
            'yaw_act':           self._yaw_act,
            'sliding_surface_x': sliding_surface_x,
            'sliding_surface_y': sliding_surface_y,
            'cmd_vel_linear_x':  self._cmd_linear_x,
            'cmd_vel_angular_z': self._cmd_angular_z,
        }
        self._writer.writerow(row)
        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info('DataLoggerNode: CSV file closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
