from ultralytics import YOLO
import cv2
model = YOLO("yolo11n-pose.pt")
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    results = model(frame)
    annotated_frame = results[0].plot()
    for keypoint_data in results[0].keypoints.data:
        right_wrist = keypoint_data[10]
        if right_wrist is not None and right_wrist[2] > 0.5:
            cv2.circle(annotated_frame,
                       (int(right_wrist[0]), int(right_wrist[1])),
                       10, (0, 0, 255), -1)
    cv2.imshow('Annotated Video Stream', annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()