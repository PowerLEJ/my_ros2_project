import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_cmd = Twist()  # Twist 메시지 객체 생성
        self.state = 'move_forward'  # 초기 상태: 전진
        time.sleep(2) # 뜸을 좀 들여줘야 할 것 같아서;;

    def move_forward(self):
        self.move_cmd.linear.x = 0.1  # 0.1 m/s 전진
        self.move_cmd.angular.z = 0.0  # 회전하지 않음
        self.publisher_.publish(self.move_cmd)
        time.sleep(1)  # 10cm 전진 후 대기 (0.1 m/s로 1초 동안 전진)

    def rotate(self):
        self.move_cmd.linear.x = 0.0  # 전진하지 않음
        self.move_cmd.angular.z = 1.0  # 초당 1라디안 회전
        self.publisher_.publish(self.move_cmd)
        time.sleep(6.28)  # 360도 회전 (1라디안/초로 약 6.28초 동안 회전)

    def move_backward(self):
        self.move_cmd.linear.x = -0.1  # 0.1 m/s 후진
        self.move_cmd.angular.z = 0.0  # 회전하지 않음
        self.publisher_.publish(self.move_cmd)
        time.sleep(1)  # 10cm 후진 후 대기 (0.1 m/s로 1초 동안 후진)

    def stop(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0  # 모든 속도 0으로 설정하여 종료
        self.publisher_.publish(self.move_cmd)

    def perform_actions(self):
        # 10cm 전진
        self.move_forward()

        # 360도 회전
        self.rotate()

        # 10cm 후진
        self.move_backward()

        # 종료
        self.stop()
        self.get_logger().info('TurtleBot has completed the movement.')

def main(args=None):
    rclpy.init(args=args)

    turtlebot_mover = TurtleBotMover()

    turtlebot_mover.perform_actions()

    rclpy.shutdown()  # 노드 종료

if __name__ == '__main__':
    main()
