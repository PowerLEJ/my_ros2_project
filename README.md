# 감성적인 터틀봇  

## 프로젝트 컨셉  
로봇이 집에서 TV(노트북 화면)를 시청하다가 감성에 젖어 가고 싶은 국가로 떠나 휴식을 취하고 집으로 돌아옵니다.  
노트북 화면은 모닥불(1 빨강), 바다(2 파랑), 사막(3 노랑), 눈(4 백), 산(5 초록) 등에 대한 사진 또는 영상물을 보여줍니다.  
로봇은 OpenCV로 10초동안 보고, 색깔을 분석을 통해 가고 싶은 여행지를 선택합니다.  
여행을 떠나기 전의 감정과 집으로 돌아오기 전의 감정을 음성으로 표현합니다.  
도착한 여행지에서 둘러보다가  
시원한 느낌이 필요한 장소에서는 선풍기(모터), 부채(서보모터)를 작동시키고  
따뜻한 느낌이 필요한 장소에서는 난로(램프)를 켭니다.  
충분한 휴식을 즐기고 나서 집(원점 0,0)으로 돌아옵니다.  


## 사양  
터틀봇3  
라즈베리파이3  
ROS2  
Foxy  
리눅스 우분투 20.04  
파이썬 3.8.10  


## 설치  

### gTTS (Google Text-to-Speech)
```
pip install gtts

sudo apt install mpg321
```  


## 라즈베리파이 핀 연결 (Physical Pin 기준)  

## GND  
39번(메모용 : 보라색 선으로 연결함)  

## 5V  
2번 (메모용 : 갈색 선으로 연결함)  

### 모터  
모터 드라이버 PWM (속도 제어) IN1 : 32번 (메모용 : 흰색 선으로 연결함)  
모터 드라이버 PWM (속도 제어) IN2 : 33번 (메모용 : 검은색 선으로 연결함)  
GND 연결  
MOTOR-A에 모터 연결  

### 서보 모터  
서보 모터(노란부분 + 노란색 선) : 12번 (메모용 : 주황색 선으로 연결함)  
5V (빨간 부분 + 빨간색 선)  
GND (갈색 부분 + 파란색 선)  

### LED  
LED + : 7번 (메모용 : 파란색 선으로 연결함)  
LED - : 저항 (330옴 갈주주)  
저항 : GND와 연결  

![001](/images/001.jpg){: width="50%" height="50%"}{: .center}  
![002](/images/002.jpg){: width="50%" height="50%"}{: .center}  
![003](/images/003.jpg){: width="50%" height="50%"}{: .center}  
![004](/images/004.jpg){: width="50%" height="50%"}{: .center}  


## 실행  

## IP 확인 및 ssh 접속  

```
nmap -sn 10.42.0.0/24
```  
 
```
ssh ubuntu@10.42.0.99
```  

### 터틀봇3에서  

브링업  
```
ros2 launch turtlebot3_bringup robot.launch.py
```  

카메라 관련  
```
ros2 run raspicam2 raspicam2_node --ros-args --params-file `ros2 pkg prefix raspicam2`/share/raspicam2/cfg/params.yaml
```  

라즈베리파이 제어 관련  
```
ros2 run rasp_project rasp_sub
```  

### 내 서버에서  

카메라 관련  
```
ros2 run rqt_image_view rqt_image_view
```  

메인 프로그램 실행  
```
python3 ~/robot_ws/src/my_project/my_project/main.py
```  

## 지도 관련  

### 터틀봇3에서  

```
ros2 launch turtlebot3_bringup robot.launch.py
```  

### 내 서버에서  

```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```  

```
ros2 run turtlebot3_teleop teleop_keyboard
```  

### 지도 저장  
```
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```  

### 지도 실행
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/my_map.yaml
```  