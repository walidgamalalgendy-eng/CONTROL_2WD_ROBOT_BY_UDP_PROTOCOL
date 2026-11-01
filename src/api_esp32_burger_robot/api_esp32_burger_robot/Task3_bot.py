import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# استدعاء الـ API واسم السيرفيس والباكيج الصحيحة
from api_esp32_burger_robot.turtlebot_api import EdugateRobotAPI
from task3_interface.srv import Task3bot3  


class DriveStraightServiceNode(Node):
    def __init__(self):
        super().__init__('drive_straight_server')

        # 1. الاتصال بالروبوت عبر الـ API
        self.get_logger().info('Connecting to Edugate Robot API for Task 4...')
        self.robot = EdugateRobotAPI(ip="192.168.4.1", local_port=9999, remote_port=8888, cpr=360.0)

        # 2. Publisher لأوامر السرعة
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. Service Server لـ Task 4 (تم التعديل هنا لـ Task3bot3)
        self.srv = self.create_service(
            Task3bot3, '/drive_straight_acc', self.handle_drive_straight
        )

        # مواصفات عجلات الروبوت لحساب المسافة من الإنكودر
        self.wheel_radius = 0.033
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        self.cpr = 360.0

        self.get_logger().info('Task 4 Service (/drive_straight_acc) is Ready!')

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

    def get_distance(self, start_a, start_b):
        """حساب المسافة المقطوعة بالمتر بناءً على فرق الإنكودر"""
        curr_a, curr_b = self.robot.get_encoder_counts()
        delta_a = abs(curr_a - start_a)
        delta_b = abs(curr_b - start_b)
        avg_ticks = (delta_a + delta_b) / 2.0
        return (avg_ticks / self.cpr) * self.wheel_circumference

    def handle_drive_straight(self, request, response):
        """معالجة طلب السيرفيس وتطبيق المراحل الثلاث (تسارع، ثبات، تباطؤ/توقف)"""
        accel_duration = request.acceleration       # زمن التسارع
        target_speed = request.constant_speed       # السرعة الثابتة
        stop_speed_val = request.stop_speed         # مسافة أو معيار التوقف

        self.get_logger().info(
            f"Received Request -> Accel: {accel_duration}s, Constant Speed: {target_speed} m/s, Stop Speed: {stop_speed_val}"
        )

        start_enc_a, start_enc_b = self.robot.get_encoder_counts()
        start_time = time.time()
        
        twist = Twist()
        traveled_distance = 0.0

        while rclpy.ok():
            current_time = time.time()
            elapsed_time = current_time - start_time

            traveled_distance = self.get_distance(start_enc_a, start_enc_b)

            if stop_speed_val > 0.0 and traveled_distance >= stop_speed_val:
                self.get_logger().info("Stop condition reached!")
                break

            current_velocity = 0.0

            # 1. مرحلة التسارع (Acceleration Phase)
            if accel_duration > 0.0 and elapsed_time < accel_duration:
                current_velocity = target_speed * (elapsed_time / accel_duration)
            # 2. مرحلة السرعة الثابتة (Constant Speed Phase)
            elif stop_speed_val <= 0.0 or traveled_distance < (stop_speed_val - 0.1):
                current_velocity = target_speed
            # 3. مرحلة التباطؤ (Deceleration Phase)
            else:
                remaining_dist = stop_speed_val - traveled_distance
                if remaining_dist > 0:
                    current_velocity = target_speed * (remaining_dist / 0.1)
                    current_velocity = max(0.05, min(current_velocity, target_speed))
                else:
                    current_velocity = 0.0

            twist.linear.x = float(current_velocity)
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            pwm = self.speed_to_pwm(current_velocity)
            self.robot.set_motor_speed(1, pwm)
            self.robot.set_motor_speed(2, pwm)

            time.sleep(0.05)

        # التوقف التام في النهاية
        self.robot.set_motor_speed(1, 0)
        self.robot.set_motor_speed(2, 0)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        # استجابة السيرفيس (Response)
        response.success = True
        response.traveled_distance = float(traveled_distance)
        response.message = f"Successfully completed motion. Traveled {traveled_distance:.2f} meters."
        
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self.robot.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveStraightServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
