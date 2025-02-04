import tkinter as tk
from test_0116_A import RobotMain
from xarm.wrapper import XArmAPI
import speech_recognition as sr
from threading import Thread
import json
import google.generativeai as genai
from gtts import gTTS
from playsound import playsound


with open('API.txt', 'r') as file:
    api_key = file.read().strip()

API_KEY=api_key
genai.configure(api_key=API_KEY)
r = sr.Recognizer()
generation_config = genai.GenerationConfig(temperature=1, response_mime_type="application/json")
model = genai.GenerativeModel('gemini-1.5-flash', generation_config=generation_config)

class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("아이스크림 토핑 선택")
        self.root.geometry("300x400")
        
        self.topping_sequence = []  # 선택된 토핑 순서를 저장
        self.topping_sequence_blueberry=[]
        self.topping_sequence_strawberry=[]
        self.text = ""  # 음성 인식 텍스트
        self.listening = False  # 음성 인식 상태
        self.stop_listening = None  # listen_in_background 반환 값 저장
        self.order_strawberry={}
        self.order_blueberry={}
        # 메인 프레임 생성
        main_frame = tk.Frame(self.root)
        main_frame.pack(pady=20)

        # 토핑 추가 버튼 및 체크박스 생성
        self.create_topping_options(main_frame)

        # 주문 버튼 생성
        order_btn = tk.Button(self.root, text="주문", command=self.order)
        order_btn.pack(pady=10)

        # 음성 인식 버튼 생성
        self.btn_text = tk.StringVar(value="음성 시작")
        voice_btn = tk.Button(self.root, textvariable=self.btn_text, command=self.toggle_listening, font=("Arial", 16))
        voice_btn.pack(pady=20)

        # 버리기 버튼 생성
        discard_btn = tk.Button(self.root, text="버리기", command=self.discard_action, font=("Arial", 16), bg="red", fg="white")
        discard_btn.pack(pady=20)

        self.root.mainloop()
    
    def speak(self, text):
        file_name = 'voice.mp3'
        tts = gTTS(text=text, lang='ko')
        tts.save(file_name)
        playsound(file_name)

    def create_topping_options(self, parent_frame):
        # 토핑 라디오 버튼 (중복 선택 불가)
        toppings = ["A", "B", "C", "N"]
        for topping in toppings:
            frame = tk.Frame(parent_frame)
            frame.pack(anchor="w", pady=5)

            # 각 토핑 추가 버튼
            add_btn = tk.Button(frame, text=f"{topping} 추가", command=lambda t=topping: self.add_topping(t))
            add_btn.pack(side="left", padx=10)

    def add_topping(self, topping):
        self.topping_sequence.append(topping)
        print(f"현재 토핑 순서: {''.join(self.topping_sequence)}")
    
    def toggle_listening(self):
        if self.listening:
            # 음성 정지
            self.listening = False
            self.btn_text.set("음성 시작")
            if self.stop_listening:
                self.stop_listening(wait_for_stop=False)
            print(f"최종 텍스트: {self.text}")
        else:
            # 음성 시작
            self.listening = True
            self.btn_text.set("음성 정지")
            self.speak('무엇을 도와 드릴까요?')

            # 음성 인식을 listen_in_background로 실행
            self.stop_listening = r.listen_in_background(sr.Microphone(), self.process_audio)

    def process_audio(self, recognizer, audio):
        try:
            recognized_text = recognizer.recognize_google(audio, language='ko')
            self.text += recognized_text
            print(f"인식된 텍스트: {recognized_text}")
        except sr.UnknownValueError:
            print("인식 실패")
        except sr.RequestError as e:
            print(f"요청 실패: {e}")
        except Exception as ex:
            print(f"예상치 못한 오류: {ex}")

    def discard_action(self):
        arm = XArmAPI('192.168.1.184', baud_checkset=False)
        robot_main = RobotMain(arm, "C")
        robot_main.Trash_Throw()

    def summarize_order(self):
        if not self.topping_sequence:
            print("선택된 주문이 없습니다!")
            self.speak("선택된 주문이 없습니다")
            return
        else:
            # A, B, C, N을 토핑 이름으로 매핑
            topping_names = {"A": "조리퐁", "B": "코코볼", "C": "해바라기씨", "N": "토핑없음"}
            topping_counts_strawberry = {"A": 0, "B": 0, "C": 0, "N": 0}
            topping_counts_blueberry = {"A": 0, "B": 0, "C": 0, "N": 0}

            # 토핑 개수 계산
            for topping in self.topping_sequence_strawberry:
                topping_counts_strawberry[topping] += 1
            for topping in self.topping_sequence_blueberry:
                topping_counts_blueberry[topping] += 1

            # 결과 문자열 생성
            result = []
            i=0
            for key, count in topping_counts_strawberry.items():
                
                if count > 0 and key != "N":  # N은 제외하고 출력
                    if i==0:
                        result.append(f"딸기아이스크림에")
                    result.append(f"{topping_names[key]} 토핑 {count}개")
                    if key=='N':
                        result.append(f"토핑없이 {count}개")
                    i+=1
            
            i=0
            for key, count in topping_counts_blueberry.items():
                
                if count > 0 :  # N은 제외하고 출력
                    if i==0:
                        result.append(f"블루베리아이스크림에")
                    result.append(f"{topping_names[key]} 토핑 {count}개")
                    if key=='N':
                        result.append(f"토핑없이 {count}개")
                    i+=1
            # 최종 메시지 출력
            if result:
                print(f"당신은 {', '.join(result)} 주문하였습니다.")
                text = f"당신은 {', '.join(result)} 주문하였습니다."
                self.speak(text)
            
    def order(self):
        self.listening = False
        if self.stop_listening:
            self.stop_listening(wait_for_stop=False)
        self.btn_text.set("음성 시작")
        if self.text != '':
            generation_config=genai.GenerationConfig(temperature=1,response_mime_type="application/json")
            model=genai.GenerativeModel('gemini-2.0-flash-exp',generation_config=generation_config)
            print("주문된 텍스트:")
            #self.text_name+='여기서 닉네임을 말해줘 "조리퐁","코코볼","해바라기씨"는 닉네임이 될수 없어'
           
            self.text+='혹시 텍스트에서 오류가 있을 수도 있으니 알아서 "조리퐁","코코볼","해바라기씨","토핑없음"으로 분류해서 토핑개수 말해줘 한국어로'
            self.text+='이 JSON 스키마를 사용해서, Order={"아이스크림":이름, "토핑":{"코코볼":개수, "조리퐁":개수, "해바라기씨":개수}}'
            self.text+='답장형식의 주 키는 "아이스크림","토핑"만 있게 해줘'
            self.text+='답장형식의 딕셔너리 키는 "토핑"에서 오직 "조리퐁", "코코볼","해바라기씨", "토핑없음" 만있게 해줘'
            self.text+='그리고 아이스크림도 무엇이 있는지 말해줘 아이스크림은 "블루베리","딸기"만 있어'
            self.text+='"블루베리"아이스크림,"딸기"아이스크림에 대한 각 토핑 값을 모두 출력하되 없으면 0으로 표시해줘'
            self.text+='예시를 보여주면: [{"아이스크림": "딸기", "토핑": {"조리퐁": "0", "코코볼": "0", "해바라기씨": "1", "토핑없음": "0"}},{"아이스크림": "블루베리", "토핑": {"조리퐁": "0", "코코볼": "0", "해바라기씨": "0", "토핑없음": "0"}}]'
            self.text+='위의 예시처럼 보여라'
            self.text+='"블루베리"에 대해서만 말해도 "딸기"에 토핑개수도 말해줘'
            #self.text+='닉네임도 말해주는데 위 토핑은 닉네임이 될수 없어 '
            #self.text+='토핑추가 없이 개수만 말하면 "토핑없음":count 로 알려줘. 이것도 없으면 0으로 말해줘'
            
            response=model.generate_content("""
            이 JSON 스키마를 사용해서, Order={"아이스크림":"딸기",{"코코볼":개수, "조리퐁":개수, "해바라기씨":개수},"아이스크림":"블루베리",{"코코볼":개수, "조리퐁":개수, "해바라기씨":개수} } 형태로만 출력하세요.
            텍스트나 설명 없이 JSON 딕셔너리만 반환하세요.
            
            """)
            
            response=model.generate_content(self.text)
            #response_name=model.generate_content(self.text_name)
            print(response.text)
            #print(response_name.text)
            self.text=''
            try:
                order = json.loads(response.text)
                sorted_list=sorted(order, key=lambda x: x["아이스크림"])
                print(sorted_list)
                self.order_strawberry=sorted_list[0]["토핑"]
                self.order_blueberry=sorted_list[1]["토핑"]
                
                
                output_string = ""
                key_to_char = {"조리퐁": "A", "코코볼": "B", "해바라기씨": "C", "토핑없음": "N"}
                for key, count in self.order_strawberry.items():
                    output_string += key_to_char.get(key, "") * int(count)
                self.topping_sequence_strawberry = list(output_string)

                output_string = ""
                for key, count in self.order_blueberry.items():
                    output_string += key_to_char.get(key, "") * int(count)
                self.topping_sequence_blueberry = list(output_string)
                
            except json.JSONDecodeError as e:
                print("JSONDecodeError:", e)
                print("Response text was not valid JSON:", response.text)

        self.topping_sequence_strawberry.sort()
        self.topping_sequence_blueberry.sort()
        self.topping_sequence=self.topping_sequence_strawberry+self.topping_sequence_blueberry
        
        if not self.topping_sequence:
            print("선택해주세요!")
            return
        
        print(self.topping_sequence)
        print('딸기',self.topping_sequence_strawberry)
        print('블루베리',self.topping_sequence_blueberry)
        self.summarize_order()

        for topping in self.topping_sequence:
            arm = XArmAPI('192.168.1.184', baud_checkset=False)
            robot_main = RobotMain(arm,topping)
            robot_main.run()
            
        self.topping_sequence_strawberry = []
        self.topping_sequence_blueberry = []
        self.topping_sequence=[]

if __name__ == "__main__":
    GUI()
