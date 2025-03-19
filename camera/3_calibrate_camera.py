import cv2
import numpy as np
import glob

chessboard_size = (7, 4)
square_size = 0.035  # 체커보드의 실제 길이 (m)

# 체커보드 기준 가상의 3차원 좌표 생성
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

images = glob.glob('captured_images/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체커보드 특징점 추출
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # 시각화
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard Corners', img)
        cv2.waitKey(5000)

cv2.destroyAllWindows()

# camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret:
    # reprojection error 를 확인하여 calibration이 잘 진행되었는지 확인 
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    # 결과 출력
    print( "reprojection error: {}".format(mean_error/len(objpoints)) )
    print("Camera matrix (mtx):")
    print(mtx)
    print("\nDistortion coefficients (dist):")
    print(dist)
    # print("\nRotation vectors (rvecs):")  # Rodrigues'
    # print(rvecs)
    # print("\nTranslation vectors (tvecs):")
    # print(tvecs)
else:
    print("Calibration failed.")

np.savez("camera_calibration_data.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
print("Calibration data saved to 'camera_calibration_data.npz'")

# def project_3d_to_2d(self, point_3d):
#     X = point_3d[0]
#     Y = point_3d[1]
#     Z = point_3d[2]
#     fx = self.intrinsic_matrix[0,0]
#     fy = self.intrinsic_matrix[1,1]
#     cs = self.intrinsic_matrix[0,2]
#     cy = self.intrinsic_matrix[1,2]
#     pixel_x = (fx * X / Z) + cs
#     pixel_y = (fy * Y / Z) + cy
#     return int(pixel_x), int(pixel_y)