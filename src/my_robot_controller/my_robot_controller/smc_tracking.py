import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped 
import math
import numpy as np

class SMCTrackingNode(Node):
    def __init__(self):
        super().__init__('smc_tracking_node')
        
        # 1. Publisher: Gửi lệnh vận tốc
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Subscriber: Nhận phản hồi vị trí (Odom)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. Tao Publisher cho Setpoint (Muc tieu mong muon)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Biến lưu trạng thái robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Timer: Chạy vòng lặp điều khiển (ví dụ 20Hz)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Thời gian bắt đầu
        self.t = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        # Lấy vị trí
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Lấy góc quay (Convert Quaternion -> Euler)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def get_desired_trajectory(self, t):
        # --- TẠO QUỸ ĐẠO HÌNH TRÒN ---
        R = 3.0  # Bán kính 1m
        omega = 0.06667 # Tốc độ góc quỹ đạo
        
        xd = R * math.cos(omega * t)
        yd = R * math.sin(omega * t)
        
        # Góc mong muốn (tiếp tuyến quỹ đạo hoặc hướng vào tâm tuỳ bài toán)
        # Ở đây giả sử muốn đầu robot luôn hướng theo đường đi
        dx = -R * omega * math.sin(omega * t)
        dy = R * omega * math.cos(omega * t)
        yaw_d = math.atan2(dy, dx)
        
        return xd, yd, yaw_d

    def control_loop(self):
        # Cập nhật thời gian
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.t = current_time - self.start_time
        
        # 1. Lấy toạ độ mong muốn
        xd, yd, yaw_d = self.get_desired_trajectory(self.t)
        
        # 2. Tính sai số (Global Frame)
        ex = xd - self.x
        ey = yd - self.y
        e_yaw = yaw_d - self.yaw
        
        # Chuẩn hoá góc e_yaw về [-pi, pi]
        e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))

        # 3. Chuyển sai số sang Body Frame (Quan trọng!)
        # Vì robot điều khiển theo hướng mũi nó, nên cần xoay sai số
        ex_b =  math.cos(self.yaw) * ex + math.sin(self.yaw) * ey
        ey_b = -math.sin(self.yaw) * ex + math.cos(self.yaw) * ey

        # 4. Áp dụng SLIDING MODE CONTROL
        # Chọn tham số Gain (K) và độ rộng lớp biên (phi)
        k_v = 0.5  # Gain cho vận tốc dài
        k_w = 1.5  # Gain cho vận tốc góc
        phi = 0.5  # Để giảm Chattering
        
        # Luật điều khiển: u = K * tanh(error / phi)
        # Sử dụng tanh để làm mượt thay vì sign()
        v_cmd_x = k_v * math.tanh(ex_b / phi)
        v_cmd_y = k_v * math.tanh(ey_b / phi)
        w_cmd_z = k_w * math.tanh(e_yaw / phi)

        # 5. Gửi lệnh
        twist = Twist()
        twist.linear.x = v_cmd_x
        twist.linear.y = v_cmd_y # Robot 4 chân có thể đi ngang (Holonomic)
        twist.angular.z = w_cmd_z
        
        self.cmd_pub.publish(twist)

        # 6 Gửi Setpoint để Visualizer
        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "odom"

        target_msg.pose.position.x = xd
        target_msg.pose.position.y = yd
        target_msg.pose.position.z = 0.0

        self.target_pub.publish(target_msg)
        
        # Debug Log
        # self.get_logger().info(f"Err: [{ex_b:.2f}, {ey_b:.2f}] -> Cmd: [{v_cmd_x:.2f}, {v_cmd_y:.2f}]")

def main(args=None):
    rclpy.init(args=args)
    node = SMCTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()