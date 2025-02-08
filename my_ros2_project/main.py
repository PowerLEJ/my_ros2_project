import subprocess
import time

def run_ros2_launch_files():
    print("1. OpenCV 분석 시작")
    subprocess.run(['ros2', 'run', 'my_project', 'watch_tv'])
    print("OpenCV 분석 완료")

    # 결과 파일 읽기
    try:
        with open('result.txt', 'r') as f:
            result = int(f.read().strip())
    except FileNotFoundError:
        print("결과 파일을 찾을 수 없음")
        result = -1
    
    print(f"분석 결과 1 : {result}")

    try:
        assert result in [-1, 0, 1, 2, 3, 4, 5]
    except AssertionError:
        print("0부터 5까지만 가능")
        result = 0
    
    print(f"분석 결과 2 : {result}")
    time.sleep(2)

    print("2. 로봇이 출발하기 전 말하기")
    subprocess.run(['ros2', 'run', 'my_project', 'robot_start_speak', str(result)])
    time.sleep(2)

    print("3. Waypoint로 이동")
    subprocess.run(['ros2', 'run', 'my_project', 'way', str(result)])
    time.sleep(2)

    print("4. move 활동")
    subprocess.run(['ros2', 'run', 'my_project', 'move'])
    time.sleep(2)

    print("5. 라즈베리파이에 데이터 전송")
    subprocess.run(['ros2', 'run', 'my_project', 'rasp_cmd', str(result)])  # 분석 결과 전달
    time.sleep(2)

    print("6. 로봇이 행동 후 말하기")
    subprocess.run(['ros2', 'run', 'my_project', 'robot_end_speak', str(result)])
    time.sleep(2)

    print("7. 원점으로 복귀")
    subprocess.run(['ros2', 'run', 'my_project', 'way', '0'])
    time.sleep(2)


if __name__ == "__main__":
    run_ros2_launch_files()
