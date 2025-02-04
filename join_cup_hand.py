import cv2
import numpy as np
import time
import mediapipe as mp
from tensorflow.keras.models import load_model

class YellowContourDetectorWithMediaPipe:
    def __init__(self):
        # HSV 색상 범위 설정
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        self.info=''
        # 카메라 설정
        self.cap = cv2.VideoCapture(2)
        self.last_warning_time = 0
        self.warning_cooldown = 0.5
        self.text='True'
        self.img=None
        # MediaPipe 손 검출 초기화
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # 모델 로드 및 클래스 이름 설정
        self.model_path = "keras_model.h5"
        self.model = load_model(self.model_path, compile=False)
        self.class_names = ['None','Yes']

        # 예측 타이머 초기화
        self.last_prediction_time = time.time()
        self.prediction_interval = 0.5  # 0.5초 간격으로 예측

    def process_frame(self):
        yellow_line_y = 300  # 노란색 라인의 Y 좌표 설정

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            self.img =frame
            frame=cv2.flip(frame,1)
            # BGR에서 HSV로 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

            # 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            mask_yellow = cv2.erode(mask_yellow, kernel, iterations=1)
            mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=4)

            # MediaPipe 손 검출
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)

            # 손 랜드마크 처리
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    h, w, c = frame.shape
                    x_min, y_min, x_max, y_max = w, h, 0, 0
                    for lm in hand_landmarks.landmark:
                        x, y = int(lm.x * w), int(lm.y * h)
                        x_min = min(x_min, x)
                        y_min = min(y_min, y)
                        x_max = max(x_max, x)
                        y_max = max(y_max, y)

                    # 손 영역 추출
                    padding = 10
                    x_min = max(x_min - padding, 0)
                    y_min = max(y_min - padding, 0)
                    x_max = min(x_max + padding, w)
                    y_max = min(y_max + padding, h)

                    hand_roi = mask_yellow[y_min:y_max, x_min:x_max]
                    yellow_pixels = cv2.countNonZero(hand_roi)
                    total_pixels = (x_max - x_min) * (y_max - y_min)

                    if total_pixels > 0:
                        yellow_ratio = yellow_pixels / total_pixels
                        if yellow_ratio > 0.02:
                            self.text='False'
                            current_time = time.time()
                            if current_time - self.last_warning_time > self.warning_cooldown:
                                print("뒤로가세요!")
                                self.last_warning_time = current_time
                        else:
                            self.text='True'  
                    else:
                        self.text='True'
                   # print(self.text)       
            
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                    cv2.putText(frame, f'Yellow Ratio: {yellow_ratio*100:.2f}%',
                                (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 0, 0), 2)
                    self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            else:
                self.text='True'
            #print(self.text)

            # 특정 시간마다 모델 예측
            # current_time = time.time()
            # #if current_time - self.last_prediction_time >= self.prediction_interval:
            # # img =frame[10:90,370:420]
            # img_input = cv2.resize(img, (224, 224))
            # img_input = cv2.cvtColor(img_input, cv2.COLOR_BGR2RGB)
            # img_input = (img_input.astype(np.float32) / 127.5) - 1
            # img_input = np.expand_dims(img_input, axis=0)

            # # 예측 수행
            # prediction = self.model.predict(img_input, verbose=0)
        
            # idx = np.argmax(prediction)
            # self.info=self.class_names[idx]
            # #print('0000000000000000000000000',self.info)
            # cv2.putText(img, text=self.class_names[idx], org=(10, 30), 
            #             fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, 
            #             color=(0, 0, 255), thickness=2)
            # self.last_prediction_time = current_time

            # 결과 출력
            cv2.imshow('Original', frame)
            
            #cv2.imshow('Yellow Mask', mask_yellow)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()


def main():
    detector = YellowContourDetectorWithMediaPipe()
    try:
        detector.process_frame()
    finally:
        detector.cleanup()


if __name__ == "__main__":
    main()
