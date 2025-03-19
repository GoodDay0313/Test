import cv2
import os

save_dir = "captured_images"
os.makedirs(save_dir, exist_ok=True)
cap = cv2.VideoCapture(0)

img_counter = 0
while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to capture image.")
        break

    cv2.imshow("Webcam", frame)

    key = cv2.waitKey(1) & 0xFF
    
    # s 눌러서 이미지 캡처
    if key == ord('s'):
        img_name = os.path.join(save_dir, f"captured_image_{img_counter}.png")
        cv2.imwrite(img_name, frame)
        print(f"Image saved as {img_name}")
        img_counter += 1

    # q 눌러서 종료
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
