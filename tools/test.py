import cv2

cap = cv2.VideoCapture('/dev/video6')
while True:
  ret, mat = cap.read()
  cv2.imshow('cap', mat)
  cv2.waitKey(1)