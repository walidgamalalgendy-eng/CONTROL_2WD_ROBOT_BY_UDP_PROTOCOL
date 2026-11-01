import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from api_esp32_burger_robot.turtlebot_api import EdugateRobotAPI

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pub_sub_node')

        self.get_logger().info('Connecting to Edugate Robot API...')
        self.robot = EdugateRobotAPI(ip="192.168.4.1", local_port=9999, remote_port=8888, cpr=360.0)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.wheel_radius = 0.033
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        self.cpr = 360.0
        self.x = 0.0
        self.y = 0.0
        self.last_enc_a, self.last_enc_b = self.robot.get_encoder_counts()

        self.create_timer(0.05, self.publish_sensor_data)
        self.get_logger().info('Pub/Sub Node is Ready! Listening to /cmd_vel and publishing to /odom & /imu')

    def speed_to_pwm(self, velocity_m_s):
        """تحويل السرعة الخطية إلى PWM من -255 إلى 255"""
        if abs(velocity_m_s) < 0.01:
            return 0
        pwm = int((velocity_m_s / 0.25) * 255.0)
        if pwm > 0:
            return max(80, min(255, pwm))
        elif pwm < 0:
            return min(-80, max(-255, pwm))
        return 0

    def cmd_vel_callback(self, msg):
        """استقبال أمر السرعة وتطبيقه على المواتير"""
        linear = msg.linear.x
        angular = msg.angular.z

        wheel_base = 0.16 
        vel_left = linear - (angular * wheel_base / 2.0)
        vel_right = linear + (angular * wheel_base / 2.0)

        pwm_left = self.speed_to_pwm(vel_left)
        pwm_right = self.speed_to_pwm(vel_right)

        # تحريك المواتير عبر الـ API
        self.robot.set_motor_speed(1, pwm_left)
        self.robot.set_motor_speed(2, pwm_right)

    def publish_sensor_data(self):
        """قراءة الحساسات ونشرها على التوبيكس"""
        curr_a, curr_b = self.robot.get_encoder_counts()
        imu_data = self.robot.get_IMU()

        # حساب الإزاحة
        d_left = ((curr_a - self.last_enc_a) / self.cpr) * self.wheel_circumference
        d_right = ((curr_b - self.last_enc_b) / self.cpr) * self.wheel_circumference
        self.last_enc_a = curr_a
        self.last_enc_b = curr_b

        d_center = (d_left + d_right) / 2.0
        yaw_rad = math.radians(imu_data['yaw'])

        self.x += d_center * math.cos(yaw_rad)
        self.y += d_center * math.sin(yaw_rad)

        # نشر /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        self.odom_pub.publish(odom_msg)

        # نشر /imu
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = float(imu_data['roll'])
        imu_msg.orientation.y = float(imu_data['pitch'])
        imu_msg.orientation.z = float(imu_data['yaw'])
        self.imu_pub.publish(imu_msg)

    def stop_robot(self):
        """إيقاف المواتير فوراً عند الخروج"""
        try:
            self.robot.set_motor_speed(1, 0)
            self.robot.set_motor_speed(2, 0)
            self.get_logger().info('Robot stopped safely.')
        except Exception:
            pass

    def destroy_node(self):
        self.stop_robot()
        self.robot.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
