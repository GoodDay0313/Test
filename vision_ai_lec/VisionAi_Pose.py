from ultralytics import YOLO
import cv2
import numpy as np

# 모델 로드 (기존 yolo11n-pose.pt 유지)
model = YOLO("yolo11n-pose.pt")

# 웹캠 연결
cap = cv2.VideoCapture(0)

# 손 위치 저장용 리스트 (손 흔들기 감지)
hand_positions = []

# 손 이동 감지 기준 거리 (x축)
movement_threshold = 50

# 최근 손 위치 추적
prev_left_hand_x = None
prev_right_hand_x = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO 모델을 사용하여 프레임 분석
    results = model(frame)
    annotated_frame = results[0].plot()

    # 검출된 사람의 keypoints 가져오기
    for result in results:
        keypoints = result.keypoints  # (N, 17, 3) -> 17개 관절의 (x, y, confidence) 좌표

        if keypoints is not None:
            for kp in keypoints:  # 여러 명이 있을 경우 반복
                kp = kp.cpu().numpy()  # 텐서를 NumPy 배열로 변환
                
                # 사람이 최소 1명이 감지되고, keypoints 배열의 길이가 17 이상인지 확인
                if len(kp) > 10:  # 적어도 10개의 키포인트가 있어야 함
                    try:
                        # 주요 관절 인덱스
                        LEFT_HAND = 9   # 왼손
                        RIGHT_HAND = 10  # 오른손
                        LEFT_ELBOW = 7   # 왼팔꿈치
                        RIGHT_ELBOW = 8  # 오른팔꿈치
                        HEAD = 0         # 머리
                        
                        # 관절 좌표 가져오기
                        left_hand = kp[LEFT_HAND]
                        right_hand = kp[RIGHT_HAND]
                        left_elbow = kp[LEFT_ELBOW]
                        right_elbow = kp[RIGHT_ELBOW]
                        head = kp[HEAD]

                        # 손목 위치 계산 (손과 팔꿈치의 중간 지점)
                        left_wrist = left_hand[:2]  # 왼손의 좌표 그대로 사용
                        right_wrist = right_hand[:2]
                        
                        # 왼손 위치에 빨간 점 표시
                        if left_hand is not None and left_hand[2] > 0.5:  # confidence threshold
                            cv2.circle(annotated_frame, (int(left_hand[0]), int(left_hand[1])), 10, (0, 0, 255), -1)  # 왼손
                        if right_hand[2] > 0.5:  # confidence threshold
                            cv2.circle(annotated_frame, (int(right_hand[0]), int(right_hand[1])), 10, (0, 0,255), -1)  # 오른손

                        # 손의 움직임 감지
                        waving_detected = False
                        
                        # 왼손과 오른손 x 좌표 저장
                        left_hand_x = left_hand[0]
                        right_hand_x = right_hand[0]

                        # 손의 이동 감지 (현재와 이전 위치 비교)
                        if prev_left_hand_x is not None:
                            # 왼손의 이동
                            left_x_movement = abs(left_hand_x - prev_left_hand_x)
                            if left_x_movement > movement_threshold:
                                waving_detected = True
                        if prev_right_hand_x is not None:
                            # 오른손의 이동
                            right_x_movement = abs(right_hand_x - prev_right_hand_x)
                            if right_x_movement > movement_threshold:
                                waving_detected = True

                        # "Wave Detected!" 메시지 표시
                        if waving_detected:
                            cv2.putText(annotated_frame, "Waving Detected!", (50, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                        # 현재 손 위치를 이전 위치로 업데이트
                        prev_left_hand_x = left_hand_x
                        prev_right_hand_x = right_hand_x

                    except IndexError:
                        # 만약 keypoints의 인덱스가 부족하면 무시하고 계속 진행
                        continue

    # 화면에 표시
    cv2.imshow('Annotated Video Stream', annotated_frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
