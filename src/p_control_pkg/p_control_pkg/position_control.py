#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

# --- Constants (Chiều dài khớp - Đã cập nhật cho ROBODOG) ---
L1 = 1.0      # Shoulder offset (m) - Giữ bằng 0.0 cho IK 2D đơn giản
L2 = 26.0 # Thigh length (m) - 5.75 mm
L3 = 20.0 # Shin length (m) - 6.329 mm

def constrain(val, low, high):
    return max(low, min(high, val))

def inverse_kinematics(x, y, z, flip=False):
    """
    Convert foot (x, y, z) position -> 3 joint angles (rad)
    """
    t1 = math.atan2(y, x)
    R = math.sqrt(x * x + y * y) - L1
    Z = z
    
    # Cosine rule
    D = (R * R + Z * Z - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    D = constrain(D, -1, 1)

    # t3 (góc shin/knee)
    # flip=False (fr, bl): knee up/forward (positive root)
    # flip=True (fl, br): knee down/backward (negative root)
    # Dựa trên trục xoay trong XACRO, có thể cần thử nghiệm lại dấu.
    t3 = math.atan2(-math.sqrt(1 - D * D) if flip else math.sqrt(1 - D * D), D)
    
    # t2 (góc leg/thigh)
    t2 = math.atan2(Z, R) - math.atan2(L3 * math.sin(t3), L2 + L3 * math.cos(t3))
    
    # Trả về góc bằng RADIAN (ROS standard)
    return [t1, t2, t3]

class SingleLegIKPublisher(Node):
    def __init__(self, leg_name='fr', flip=False):
        super().__init__('single_leg_ik_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.leg_name = leg_name.lower()
        self.flip = flip
        self.timer = self.create_timer(0.02, self.timer_callback) 

        # Tọa độ thử nghiệm: Trong workspace (khoảng 0.001 m - 0.012 m)
        # Z âm để chân duỗi xuống
        self.x = 0.006 
        self.y = 0.000 
        self.z = -0.008 
        self.get_logger().info(f'🦿 Đang điều khiển chân: {self.leg_name.upper()} | Chiều dài: L2={L2:.4f}m, L3={L3:.4f}m')

    def timer_callback(self):
        # Tính toán góc khớp theo IK (trả về RADIAN)
        angles_rad = inverse_kinematics(self.x, self.y, self.z, self.flip)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # CẬP NHẬT TÊN JOINT MỚI DỰA TRÊN XACRO: robodog.xacro
        leg_prefix = f'{self.leg_name}_'
        
        # Tên khớp trong XACRO:
        # 1. Thigh: <leg>_thigh_link_to_base_link
        # 2. Leg: <leg>_leg_link_to_<leg>_thigh_link
        # 3. Shin: <leg>_shin_link_to_<leg>_leg_link
        
        # LƯU Ý: Khớp thứ 2 và 3 có tên hơi khác so với quy ước chung, cần ánh xạ lại
        
        # Ánh xạ theo thứ tự IK: t1, t2, t3
        if self.leg_name in ['fr', 'br']:
            # Chân phải: fr_thigh_link_to_base_link (t1), fr_leg_link_to_fr_thigh_link (t2), fr_shin_link_to_fr_leg_link (t3)
            msg.name = [f'{self.leg_name}_thigh_link_to_base_link',
                        f'{self.leg_name}_leg_link_to_{self.leg_name}_thigh_link',
                        f'{self.leg_name}_shin_link_to_{self.leg_name}_leg_link']
        elif self.leg_name in ['fl', 'bl']:
            # Chân trái: fl_thigh_link_to_base_link (t1), fl_leg_link_to_fl_thigh_link (t2), fl_shin_link_to_fl_leg_link (t3)
            msg.name = [f'{self.leg_name}_thigh_link_to_base_link',
                        f'{self.leg_name}_leg_link_to_{self.leg_name}_thigh_link',
                        f'{self.leg_name}_shin_link_to_{self.leg_name}_leg_link']
        else:
            self.get_logger().error(f"Tên chân '{self.leg_name}' không hợp lệ.")
            return

        msg.position = angles_rad 
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)

        # Chuyển đổi sang độ để hiển thị ra console
        angles_deg = [math.degrees(a) for a in angles_rad]

        self.get_logger().info(
            f"→ Chân {self.leg_name.upper()} tới (x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}) | Góc: {['%.1f°' % a for a in angles_deg]}"
        )

def main(args=None):
    rclpy.init(args=args)

    # --- Nhập chân và tọa độ ---
    leg = input("Nhập tên chân (fr, fl, br, bl): ").strip().lower()
    
    # Dựa trên trục xoay trục 1 (axis xyz=1 0 0 hoặc -1 0 0), chân trái thường flip.
    # Tuy nhiên, ta dùng flip cho hướng gối t3. Cần kiểm tra lại dấu t3.
    # Thử nghiệm với logic cũ:
    flip = leg in ['fl', 'br'] 

    try:
        x = float(input("Nhập x (m): "))
        y = float(input("Nhập y (m): "))
        z = float(input("Nhập z (m): "))
    except ValueError:
        print("❌ Lỗi: giá trị nhập không hợp lệ.")
        return

    node = SingleLegIKPublisher(leg_name=leg, flip=flip)
    node.x, node.y, node.z = x, y, z

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

