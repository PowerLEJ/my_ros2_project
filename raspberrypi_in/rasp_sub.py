# 터틀봇3 라즈베리파이 안에 들어가는 코드  
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

# GPIO 핀 설정 (BOARD 모드)
MOTOR_PWM1 = 32  # 모터 드라이버 PWM (속도 제어) - Physical Pin 32 (GPIO12)
MOTOR_PWM2 = 33  # 모터 드라이버 PWM (속도 제어) - Physical Pin 33 (GPIO13)
SERVO_PIN = 12   # 서보모터 - Physical Pin 12 (GPIO18)
LAMP_PIN = 7     # 램프(릴레이) - Physical Pin 7 (GPIO4)

# GPIO 초기 설정
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PWM1, GPIO.OUT)
GPIO.setup(MOTOR_PWM2, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(LAMP_PIN, GPIO.OUT)

# PWM 객체 생성
motor1 = GPIO.PWM(MOTOR_PWM1, 1000)
motor2 = GPIO.PWM(MOTOR_PWM2, 1000)
servo = GPIO.PWM(SERVO_PIN, 50)  # 서보모터는 50Hz 사용

# PWM 시작 (모터는 0% 속도로 시작)
motor1.start(0)
motor2.start(0)
servo.start(0)

class RaspSubscriber(Node):
    def __init__(self):
        super().__init__('rasp_subscriber')
        self.subscription = self.create_subscription(
            Int32, 'rasp_control', self.listener_callback, 10)

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        if command == 2 or command == 5:
            self.run_motor()
        elif command == 1 or command == 3:
            self.run_servo()
        elif command == 4:
            self.run_lamp()
        else:
            self.get_logger().warn("Invalid command received")

    def run_motor(self):
        print("모터 정방향 회전")
        motor1.ChangeDutyCycle(50)
        motor2.ChangeDutyCycle(0)
        time.sleep(2)
        print("모터 정지")
        motor1.ChangeDutyCycle(0)
        motor2.ChangeDutyCycle(0)

    def run_servo(self):
        print("서보모터 90도 이동")
        duty = 2 + (90 / 18)
        servo.ChangeDutyCycle(duty)
        time.sleep(0.5)
        servo.ChangeDutyCycle(0)

    def run_lamp(self):
        print("램프 ON")
        GPIO.output(LAMP_PIN, GPIO.HIGH)
        time.sleep(5)
        print("램프 OFF")
        GPIO.output(LAMP_PIN, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = RaspSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료 중...")
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()
        print("GPIO 정리 완료")

if __name__ == '__main__':
    main()
