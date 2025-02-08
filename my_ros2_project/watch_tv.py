#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
import cv2
import numpy as np
import time

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # 카메라 이미지 구독
        self.create_subscription(
            CompressedImage, 
            '/camera/image/compressed',  # 압축된 이미지 토픽
            self.image_callback, 
            10
        )
        
        # 결과 퍼블리시
        self.publisher_ = self.create_publisher(Int32, 'image_result', 10)
        
        # 분석 시간 설정
        self.start_time = time.time()
        self.results = {
            1: 0,  # 빨간색 (모닥불) - 서보모터 1
            2: 0,  # 파란색 (바다) - 모터 1
            3: 0,  # 노란색 (사막) - 서보모터 2
            4: 0,  # 흰색 (눈) - 램프
            5: 0,  # 초록색 (산) - 모터 2
            -1: 0  # 인식 실패
        }

    def image_callback(self, msg):
        # 10초 동안 이미지를 분석하고, 이후에 가장 많이 감지된 색을 선택
        if time.time() - self.start_time < 10:
            try:
                # 압축된 이미지 데이터를 OpenCV 형식으로 변환
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                # BGR → HSV 변환
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

                # 색상 범위 정의 (HSV)
                red_lower1, red_upper1 = np.array([0, 100, 100]), np.array([10, 255, 255])
                red_lower2, red_upper2 = np.array([170, 100, 100]), np.array([180, 255, 255])
                blue_lower, blue_upper = np.array([100, 100, 100]), np.array([130, 255, 255])
                yellow_lower, yellow_upper = np.array([20, 100, 100]), np.array([40, 255, 255])
                white_lower, white_upper = np.array([0, 0, 200]), np.array([180, 50, 255])  # 흰색
                green_lower, green_upper = np.array([35, 100, 100]), np.array([85, 255, 255])  # 초록색

                # 마스크 생성
                red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
                red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
                red_mask = red_mask1 + red_mask2  # 빨간색 범위 결합
                blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
                yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
                white_mask = cv2.inRange(hsv, white_lower, white_upper)  # 흰색
                green_mask = cv2.inRange(hsv, green_lower, green_upper)  # 초록색

                # 픽셀 개수 비교하여 색상 판단
                red_count = np.sum(red_mask)
                blue_count = np.sum(blue_mask)
                yellow_count = np.sum(yellow_mask)
                white_count = np.sum(white_mask)
                green_count = np.sum(green_mask)

                # 가장 많이 감지된 색 선택
                if red_count > max(blue_count, yellow_count, white_count, green_count):
                    result = 1  # 빨간색 (모닥불)
                elif blue_count > max(red_count, yellow_count, white_count, green_count):
                    result = 2  # 파란색 (바다)
                elif yellow_count > max(red_count, blue_count, white_count, green_count):
                    result = 3  # 노란색 (사막)
                elif white_count > max(red_count, blue_count, yellow_count, green_count):
                    result = 4  # 흰색 (눈)
                elif green_count > max(red_count, blue_count, yellow_count, white_count):
                    result = 5  # 초록색 (산)
                else:
                    result = -1  # 인식 실패

                # 결과 카운팅
                self.results[result] += 1

            except Exception as e:
                self.get_logger().error(f"Image processing error: {e}")

        # 10초 후 가장 많이 감지된 색 선택
        if time.time() - self.start_time >= 10:
            # 가장 많이 감지된 색을 결정
            final_result = max(self.results, key=self.results.get)

            # 결과를 파일에 저장
            with open('result.txt', 'w') as f:
                f.write(str(final_result))

            self.get_logger().info(f"Final detected: {final_result}")
            sys.exit(0)  # 10초 후 종료

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료~~~")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
