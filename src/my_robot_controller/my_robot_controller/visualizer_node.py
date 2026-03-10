import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class RealTimePlotter(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        # Đăng ký nhận dữ liệu Thực tế (PV) và Mong muốn (SP)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_callback, 10)
        
        # Dữ liệu để vẽ
        self.x_data = []
        self.y_data = []      # Robot Path
        
        self.xd_data = []
        self.yd_data = []     # Desired Path (Setpoint)
        
        self.lock = threading.Lock() # Khóa luồng để tránh xung đột dữ liệu

    def odom_callback(self, msg):
        with self.lock:
            self.x_data.append(msg.pose.pose.position.x)
            self.y_data.append(msg.pose.pose.position.y)
            
            # Giới hạn dữ liệu để không bị lag (lưu 500 điểm gần nhất)
        #    if len(self.x_data) > 500:
         #       self.x_data.pop(0)
         #       self.y_data.pop(0)

    def target_callback(self, msg):
        with self.lock:
            self.xd_data.append(msg.pose.position.x)
            self.yd_data.append(msg.pose.position.y)
            
            # if len(self.xd_data) > 500:
            #    self.xd_data.pop(0)
             #   self.yd_data.pop(0)

# Hàm cập nhật đồ thị cho Matplotlib
def update_plot(frame, node, line_pv, line_sp):
    # Lấy dữ liệu an toàn từ Node
    with node.lock:
        line_pv.set_data(node.x_data, node.y_data)
        line_sp.set_data(node.xd_data, node.yd_data)
    
    return line_pv, line_sp

def main():
    rclpy.init()
    node = RealTimePlotter()
    
    # Chạy ROS 2 trong một luồng riêng để không chặn đồ thị
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Cấu hình Matplotlib
    fig, ax = plt.subplots()
    ax.set_title("SMC Trajectory Tracking: Setpoint vs Process Value")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True)
    ax.set_aspect('equal') # Tỉ lệ 1:1 để hình tròn không bị méo
    
    # Thiết lập giới hạn trục (tùy chỉnh theo quỹ đạo của bạn)
    ax.set_xlim(-4.0, 4.0)
    ax.set_ylim(-4.0, 4.0)
    
    # Khởi tạo 2 đường: Thực tế (Xanh) và Mong muốn (Đỏ nét đứt)
    line_pv, = ax.plot([], [], 'b-', label='Robot Position (PV)', linewidth=2)
    line_sp, = ax.plot([], [], 'r--', label='Desired Path (SP)', linewidth=1.5)
    ax.legend()

    # Chạy Animation (Cập nhật mỗi 100ms)
    ani = FuncAnimation(fig, update_plot, fargs=(node, line_pv, line_sp), interval=100)
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()