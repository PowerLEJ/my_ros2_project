from gtts import gTTS
import os
import sys

def speak(text):
    # espeak 사용 (오프라인 환경 가능. 근데 말투가 부자연스러움)
    # os.system(f'espeak -v ko "{text}"')

    # gTTS (Google Text-to-Speech) 사용 (온라인 환경이어야 함)
    tts = gTTS(text=text, lang='ko') # 한국어
    file_path = "/home/lej/output.mp3"
    tts.save(file_path) # tts.save("output.mp3")
    os.system("mpg321 /home/lej/output.mp3")  # mpg321으로 음성 파일 재생
    # os.system("ffplay -nodisp -autoexit -af atempo=1.5 output.mp3")  # 속도를 1.5배로 조정

def main():
    if len(sys.argv) > 1:
        result = sys.argv[1]
    else:
        result = "0"

    text_map = {
        "0": "원점으로 이동합니다.",
        "1": "모닥불을 보니 캠핑을 가고 싶어.",
        "2": "바다를 보니 하와이에 가고 싶네.",
        "3": "사막에서 낙타를 타고 싶어.",
        "4": "눈을 보니 따뜻하게 난로를 피우고 싶어.",
        "5": "산을 보니 등산을 하며 리프레시 하고 싶어."
    }

    speech_text = text_map.get(result, "알 수 없는 명령을 받았습니다.")
    print(f"로봇이 말함: {speech_text}")
    speak(speech_text)

if __name__ == "__main__":
    main()
