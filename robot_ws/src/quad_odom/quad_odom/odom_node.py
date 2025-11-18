import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math


class QuadOdom(Node):
    def __init__(self):
        super().__init__('quad_odom')

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Velocities from cmd_vel
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_time = self.get_clock().now()

        # Publishers & subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer 50 Hz
        self.timer = self.create_timer(0.02, self.update_odom)

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate motion
        self.x += self.vx * math.cos(self.yaw) * dt - self.vy * math.sin(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt + self.vy * math.cos(self.yaw) * dt
        self.yaw += self.wz * dt

        # Quaternion
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = QuadOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
