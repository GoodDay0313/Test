import cv2
import numpy as np

image =cv2.imread('images.png')
print(image)
print(image.shape)
cv2.imshow('image', image)

# # # 흑백 이미지
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# cv2.imshow('grayscale image', gray)
# print(gray.shape)

# 더하기
# image+=50
# cv2.imshow('image1', image)

# # # 특정 rgb 값에 빼기
# image[:,:,1] -=100
# cv2.imshow('image2', image)

# 특정 행에 대해 더하기
# image[:30] += 100
# cv2.imshow('image3', image)

# 특정 열에 대해 더하기
image[:,:30] += 100
cv2.imshow('image4', image)

cv2.waitKey(0)