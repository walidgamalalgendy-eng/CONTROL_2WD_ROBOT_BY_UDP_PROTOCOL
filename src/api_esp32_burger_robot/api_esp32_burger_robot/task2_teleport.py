import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
--------------------------------------------------
Edugate TurtleBot3 Teleop Keyboard (Task 3 / Teleop):
   w : Move Forward 
   s : Move Backward 
   a : Turn Left 
   d : Turn Right 
   q : Stop 
   e : Toggle Speed 
   
   CTRL-C to quit
--------------------------------------------------
"""

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed = 0.08
        self.turn = 0.5
        self.high_speed = False
        
        print(MSG)
        self.print_current_speeds()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key.lower()

    def print_current_speeds(self):
        print(f"\rCurrent Speed: linear={self.speed:.2f} | angular={self.turn:.2f}   ", end="")

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    twist.linear.x = self.speed
                    twist.angular.z = 0.0
                    print("\rMoving Forward...       ", end="")
                elif key == 's':
                    twist.linear.x = -self.speed
                    twist.angular.z = 0.0
                    print("\rMoving Backward...      ", end="")
                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = self.turn
                    print("\rTurning Left...         ", end="")
                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -self.turn
                    print("\rTurning Right...        ", end="")
                elif key == 'q':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print("\rRobot Stopped!          ", end="")
                elif key == 'e':
                    self.high_speed = not self.high_speed
                    self.speed = 0.25 if self.high_speed else 0.15
                    self.print_current_speeds()
                    continue
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            # إيقاف الروبوت عند الخروج
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
