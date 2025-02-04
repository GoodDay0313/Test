import cv2
import mediapipe as mp
import numpy as np

class FaceMeshDetector:
    def __init__(self):
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, min_detection_confidence=0.5)
        self.previous_turn = None

    def calculate_direction(self, keypoints):
        nose_tip = keypoints[1]
        left_nose = keypoints[279]
        right_nose = keypoints[49]

        midpoint = np.array([(left_nose[0] + right_nose[0]) / 2,
                              (left_nose[1] + right_nose[1]) / 2,
                              (left_nose[2] + right_nose[2]) / 2])

        perpendicular_up = np.array([midpoint[0], midpoint[1] - 50, midpoint[2]])

        yaw = self.get_angle_between_lines(midpoint, nose_tip, perpendicular_up)
        turn = self.get_angle_between_lines(midpoint, right_nose, nose_tip)

        return yaw, turn

    def get_angle_between_lines(self, midpoint, point1, point2):
        vector1 = np.array(point1) - np.array(midpoint)
        vector2 = np.array(point2) - np.array(midpoint)

        dot_product = np.dot(vector1, vector2)
        magnitude1 = np.linalg.norm(vector1)
        magnitude2 = np.linalg.norm(vector2)

        cosine_theta = dot_product / (magnitude1 * magnitude2)
        angle_in_degrees = np.degrees(np.arccos(cosine_theta))

        return angle_in_degrees

    def draw_face_mesh(self, image, keypoints):
        for i, landmark in enumerate(keypoints):
            x = int(landmark.x * image.shape[1])
            y = int(landmark.y * image.shape[0])
            cv2.circle(image, (x, y), 1, (0, 255, 255), -1)  # Draw the landmark point
            cv2.putText(image, str(i), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

    def run(self):
        cap = cv2.VideoCapture(0)  # Capture video from the webcam

        while cap.isOpened():
            ret, image = cap.read()
            if not ret:
                break

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(image_rgb)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    #self.draw_face_mesh(image, face_landmarks.landmark)
                    keypoints = [(landmark.x, landmark.y, landmark.z) for landmark in face_landmarks.landmark]

                    yaw, turn = self.calculate_direction(keypoints)

                    # Calculate the difference from the previous turn value
                    # if self.previous_turn is not None:
                    #     turn_difference = turn - self.previous_turn
                    #     cv2.putText(image, f'Turn Difference: {turn_difference:.2f}°', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

                    # Update the previous turn value
                    self.previous_turn = turn

                    cv2.putText(image, f'Yaw: {yaw:.2f}° Turn: {turn:.2f}°', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow('Face Mesh', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on 'q' key
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = FaceMeshDetector()
    detector.run()
