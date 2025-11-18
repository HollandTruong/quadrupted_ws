#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from math import radians

class MPUSerialNode(Node):
    def __init__(self):
        super().__init__("mpu_serial")
        self.pub = self.create_publisher(Imu, "/mpu/data", 10)

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz

    def read_serial(self):
        raw = self.ser.readline()
        try:
            line = raw.decode(errors="ignore").strip()
        except:
            return

        if len(line) == 0:
            return

        try:
            ax, ay, az, gx, gy, gz, temp = map(float, line.split())
        except:
            # nếu không parse được thì bỏ qua dòng đó
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = radians(gx)
        msg.angular_velocity.y = radians(gy)
        msg.angular_velocity.z = radians(gz)

        self.pub.publish(msg)
        self.get_logger().info(f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) | "
                       f"Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) | Temp: {temp:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MPUSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
