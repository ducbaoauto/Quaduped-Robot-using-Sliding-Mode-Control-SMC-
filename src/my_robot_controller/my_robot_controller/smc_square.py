import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np

class SMCSquareNode(Node):
    def __init__(self):
        super().__init__('smc_square_node')
        
        # 1. Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 2. Biến trạng thái
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        
        # 3. THAM SỐ SMC 
        self.k_v = 0.5
        self.k_w = 1.5
        self.phi = 0.3
        
        # 4. CẤU HÌNH HÌNH VUÔNG BO GÓC (Rounded Square)
        self.square_side = 5.0   # Cạnh hình vuông 
        self.corner_radius = 1.0 # Bán kính góc bo
        self.v_des = 0.2         # Vận tốc mong muốn (m/s)
        
        # Tính toán hình học
        # Phần thẳng trong mỗi cạnh (trừ đi 2 đầu bo tròn)
        self.straight_len = self.square_side - 2 * self.corner_radius
        # Chiều dài cung tròn (1/4 hình tròn)
        self.arc_len = (2 * math.pi * self.corner_radius) / 4
        # Tổng chu vi
        self.perimeter = 4 * (self.straight_len + self.arc_len)
        
        # Timer 50Hz
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.curr_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def get_desired_state(self, t):
        """Tính toán vị trí (x,y,yaw) và vận tốc góc (w_ff) mong muốn"""
        
        # 1. Tính quãng đường đi được trên chu vi
        distance = (t * self.v_des) % self.perimeter
        
        # 2. Xác định đang ở cạnh nào (0: Đáy, 1: Phải, 2: Trên, 3: Trái)
        # Một segment = 1 đoạn thẳng + 1 góc bo
        segment_len = self.straight_len + self.arc_len
        current_segment = int(distance // segment_len) # 0, 1, 2, 3
        dist_in_segment = distance % segment_len
        
        # Logic tính toán cục bộ (Local)
        # Giả sử bắt đầu từ gốc (0,0) đi thẳng trục X rồi rẽ trái
        local_x, local_y, local_yaw = 0.0, 0.0, 0.0
        w_feedforward = 0.0
        
        if dist_in_segment < self.straight_len:
            # --- ĐI THẲNG ---
            local_x = dist_in_segment
            local_y = 0.0
            local_yaw = 0.0
            w_feedforward = 0.0
        else:
            # --- VÀO CUA ---
            arc_dist = dist_in_segment - self.straight_len
            angle = arc_dist / self.corner_radius
            
            # Công thức cung tròn tiếp tuyến với đoạn thẳng tại (straight_len, 0)
            local_x = self.straight_len + self.corner_radius * math.sin(angle)
            local_y = self.corner_radius - self.corner_radius * math.cos(angle)
            local_yaw = angle
            w_feedforward = self.v_des / self.corner_radius # Omega = v / R

        # 3. Biến đổi tọa độ Global (Dịch chuyển và Xoay mảnh ghép)
        # Để hình vuông cân đối tâm O(0,0), ta bắt đầu từ điểm:
        # X = -half_straight (lệch trái so với tâm cạnh đáy)
        # Y = -half_side (nằm ở đáy)
        
        half_side = self.square_side / 2.0
        half_straight = self.straight_len / 2.0
        
        # Tọa độ điểm bắt đầu của từng cạnh (Bottom, Right, Top, Left)
        # Và góc xoay của cạnh đó
        transforms = [
            # Start X, Start Y, Base Angle
            (-half_straight, -half_side, 0.0),          # Cạnh đáy (đi sang phải)
            (half_side, -half_straight, math.pi/2),     # Cạnh phải (đi lên)
            (half_straight, half_side, math.pi),        # Cạnh trên (đi sang trái)
            (-half_side, half_straight, -math.pi/2)     # Cạnh trái (đi xuống)
        ]
        
        base_x, base_y, base_angle = transforms[current_segment]
        
        # Công thức xoay 2D
        # x_global = x_base + x_local * cos(theta) - y_local * sin(theta)
        # y_global = y_base + x_local * sin(theta) + y_local * cos(theta)
        
        xd = base_x + local_x * math.cos(base_angle) - local_y * math.sin(base_angle)
        yd = base_y + local_x * math.sin(base_angle) + local_y * math.cos(base_angle)
        yaw_d = base_angle + local_yaw
        
        return xd, yd, yaw_d, w_feedforward

    def control_loop(self):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # 1. Lấy mục tiêu từ bộ tạo quỹ đạo
        xd, yd, yaw_d, w_ff = self.get_desired_state(t)
        
        # 2. Tính sai số Global
        ex = xd - self.curr_x
        ey = yd - self.curr_y
        e_yaw = math.atan2(math.sin(yaw_d - self.curr_yaw), math.cos(yaw_d - self.curr_yaw))
        
        # 3. Chuyển sai số sang Body Frame
        error_body_x = math.cos(self.curr_yaw) * ex + math.sin(self.curr_yaw) * ey
        error_body_y = -math.sin(self.curr_yaw) * ex + math.cos(self.curr_yaw) * ey
        
        # 4. SMC Control Law (+ Feedforward)
        v_ff = self.v_des
        
        v_x = v_ff + (self.k_v * np.tanh(error_body_x / self.phi))
        v_y = self.k_v * np.tanh(error_body_y / self.phi)
        w_z = w_ff + (self.k_w * np.tanh(e_yaw / self.phi))
        
        # 5. Gửi lệnh
        cmd = Twist()
        cmd.linear.x = float(v_x)
        cmd.linear.y = float(v_y)
        cmd.angular.z = float(w_z)
        self.cmd_pub.publish(cmd)
        
        # 6. Visualize Setpoint
        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "odom"
        target_msg.pose.position.x = xd
        target_msg.pose.position.y = yd
        self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SMCSquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()