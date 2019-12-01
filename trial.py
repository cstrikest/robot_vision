import cv2
import numpy as np
import time


readlower = np.array([156, 179, 144])
readupper = np.array([180, 255, 255])
readlower1 = np.array([0, 128, 146])
readupper2 = np.array([5, 255, 255])
lowerarry = [[readlower, readupper, 'red'], [readlower1, readupper2, 'red1']]
capture = cv2.VideoCapture(0)
while True:
	ret, frame = capture.read()
	frame = cv2.resize(frame, (640, 480))
	if ret == False:
		print("video is erro")
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	for colormin, colermax, name in lowerarry:
		mask = cv2.inRange(hsv, colormin, colermax)
	# res = cv2.bitwise_and(frame, frame, mask=mask)
	# mask=cv2.erode(mask,None,iterations=1)
	mask = cv2.dilate(mask, None, iterations = 25)
	ret, binary = cv2.threshold(mask, 15, 255, cv2.THRESH_BINARY)
	cv2.imshow('result', binary)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
	closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
	cv2.imshow('closed', closed)
	# erode = cv2.erode(closed, None, iterations=4)
	# cv2.imshow('erode', erode)
	dilate = cv2.dilate(closed, None, iterations = 50)
	cv2.imshow('dilate', dilate)
	contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# res=_.copy()
	for con in contours:
		x, y, w, h = cv2.boundingRect(con)  # 将轮廓分解为识别对象的左上角坐标和宽、高
		# 在图像上画上矩形（图片、左上角坐标、右下角坐标、颜色、线条宽度）
		cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
	
	cv2.imshow('res', frame)
	key = cv2.waitKey(1)
	if key == ord('q'):
		break