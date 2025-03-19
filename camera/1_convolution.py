import cv2
import numpy as np

image =cv2.imread('images.png')

# convolution연산을 통한 다양한 filtering
# 실행 전에 어떤 역할을 하는 filter일지 예측해보기

filter = np.array([[0.04, 0.04, 0.04, 0.04, 0.04],
                   [0.04, 0.04, 0.04, 0.04, 0.04],
                   [0.04, 0.04, 0.04, 0.04, 0.04],
                   [0.04, 0.04, 0.04, 0.04, 0.04],
                   [0.04, 0.04, 0.04, 0.04, 0.04]])

# filter = np.array([[1, 1, 1],
#                    [0, 0, 0],
#                    [-1, -1, -1]])

# filter = np.array([[1, 0, -1],
#                    [0, 0, -0],
#                    [-1, 0, 1]])


# filter = np.array([[-1, 1, 1],
#                    [-1, 0, 1],
#                    [-1, -1, 1]])

blured = cv2.filter2D(image, -1, filter)

cv2.imshow('image', image)

cv2.imshow('blured image', blured)
cv2.waitKey(0)