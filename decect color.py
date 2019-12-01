import cv2
import numpy as np
import configparser

def write_color_rangeHSV(filepath, color, upper_range, lower_range):
	config = configparser.ConfigParser()
	config.add_section(color)
	config.read(filepath, encoding = "utf-8")
	char = ['h', 's', 'v']
	for i in range(0, 3):
		config.set(color, "upper_" + char[i], str(upper_range[i]))
	for i in range(0, 3):
		config.set(color, "lower_" + char[i], str(lower_range[i]))
	config.write(open(filepath, "w"))

def nothing(x):
	pass

def createbars():
	cv2.createTrackbar("H_l", "image", 0, 180, nothing)
	cv2.createTrackbar("H_h", "image", 0, 180, nothing)
	cv2.createTrackbar("S_l", "image", 0, 255, nothing)
	cv2.createTrackbar("S_h", "image", 0, 255, nothing)
	cv2.createTrackbar("V_l", "image", 0, 255, nothing)
	cv2.createTrackbar("V_h", "image", 0, 255, nothing)

#MAIN PROGRAM
# cap = cv2.VideoCapture("./3d_red_ball.mov")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cv2.namedWindow("image")
createbars()

lower = np.array([0, 0, 0])
upper = np.array([0, 0, 0])
while True:
	ret, frame = cap.read()
	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower[0] = cv2.getTrackbarPos("H_l", "image")
	upper[0] = cv2.getTrackbarPos("H_h", "image")
	lower[1] = cv2.getTrackbarPos("S_l", "image")
	upper[1] = cv2.getTrackbarPos("S_h", "image")
	lower[2] = cv2.getTrackbarPos("V_l", "image")
	upper[2] = cv2.getTrackbarPos("V_h", "image")
	
	mask = cv2.inRange(hsv_frame, lower, upper)
	cv2.imshow("img", frame)
	cv2.imshow("mask", mask)
	if cv2.waitKey(100) == ord("q"):
		write_color_rangeHSV("./config.ini", "redBall", upper, lower)
		break

cv2.destroyAllWindows()

