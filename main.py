#!/usr/bin/env python3

__author__ = "Yxzh"

import configparser
import cv2
import numpy as np
import math

# ------Init
# ---HSV color data func
def read_color_rangeHSV(filepath, color):
	config = configparser.ConfigParser()
	config.read(filepath, encoding = "utf-8")
	ball_color_range = np.array(config.items(color))
	upper_range = ball_color_range[:3, 1].astype(np.int32).reshape(1, 3)
	lower_range = ball_color_range[3:, 1].astype(np.int32).reshape(1, 3)
	config.clear()
	return upper_range, lower_range

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

def exit_cam(cam):
	cam.release()
	cv2.destroyAllWindows()

# ---PID controller class
class PID:
	def __init__(self, P, I, D, target, interval, max_history_bias):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.interval = interval
		self.target = target
		self.bias = [0, 0]
		self.max_history_bias = max_history_bias
	
	def clear(self):
		self.bias = [0, 0]
	
	def update(self, feedback):
		self.bias.append(self.target - feedback)
		if len(self.bias) > self.max_history_bias:
			self.bias.pop(0)
		return self.Kp * (self.bias[-1]) + self.Ki * (sum(self.bias)) + self.Kd * (self.bias[-1] - self.bias[-2])

# ------MAIN PROGRAM
# ---Open camera

# cam = cv2.VideoCapture("./3d_red_ball.mov")
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cam.isOpened():
	exit_cam(cam)
	print("Unable to initialize camera.")
	exit(1)
	
# ---Set windows
# cv2.namedWindow("mask", 0)
cv2.namedWindow("camera", 0)
# cv2.resizeWindow("mask", 1920, 1080)
cv2.resizeWindow("camera", 1440, 1080)

# ---Set Vars
controller = PID(0.5, 0.01, 0.1, 0, 0.05, 50)
pos = 0
dis = 0
motor_para = 0
L_motor_para = 0
R_motor_para = 0
loss_target_count = 0
target_loss_frame = 5
is_target = False

# ---Main program loop
while True:
	ret, frame = cam.read()
	if not ret:
		exit_cam(cam)
		print("Unable to read frame. (Maybe the video is over?)")
		exit(0)
	
	# ---Image process
	frame_Blur = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	upper, lower = read_color_rangeHSV("./config.ini", "redBall")
	mask = cv2.inRange(frame_Blur, lower, upper)
	mask = cv2.medianBlur(mask, 9)
	mask = cv2.erode(mask, None, iterations = 3)
	mask = cv2.dilate(mask, None, iterations = 8)
	ret, binary = cv2.threshold(mask, 15, 255, cv2.THRESH_BINARY)
	contours, hierarchy = cv2.findContours(binary.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	# ---Target detected
	if len(contours) > 0:
		if not is_target:
			is_target = True
			controller.clear()
		loss_target_count = 0
		for i in range(0, len(contours)):
			x, y, w, h = cv2.boundingRect(contours[i])
			if w * h > 300 and hierarchy[0][i][2] == -1:
				current_father = hierarchy[0][i][2]
				moment = cv2.moments(contours[i])
				center = (int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"]))
				cv2.circle(frame, center, 3, (0, 255, 0), 2)
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
				cv2.line(frame, center, (320, center[1]), (255, 0, 0), 1)
				dis = 10 / math.sqrt(w * h)
				pos = center[0] - 320
				cv2.putText(frame, "pos: %d" % (center[0] - 320), (x, y - 5), cv2.FONT_ITALIC, 0.3, (0, 255, 0), 1)
				cv2.putText(frame, "dis: %.3f" % dis, (x, y - 17), cv2.FONT_ITALIC, 0.3, (0, 255, 0), 1)
	elif is_target:
		loss_target_count += 1
		if loss_target_count > target_loss_frame:
			loss_target_count = 0
			is_target = False
	
	# ---Draw info
	cv2.line(frame, (320, 0), (320, 1000), (0, 0, 0), 1)
	cv2.putText(frame, "Range: ", (3, 10), cv2.FONT_ITALIC, 0.4, (0, 0, 0), 1)
	cv2.putText(frame, "Upper : " + str(upper[0][0]) + " " + str(upper[0][1]) + " " + str(upper[0][2]), (15, 22),
	            cv2.FONT_ITALIC, 0.4, (0, 0, 0), 1)
	cv2.putText(frame, "Lower : " + str(lower[0][0]) + " " + str(lower[0][1]) + " " + str(lower[0][2]), (15, 34),
	            cv2.FONT_ITALIC, 0.4, (0, 0, 0), 1)
	if is_target:
		motor_para = controller.update(pos)
		cv2.putText(frame, "PID out: " + str(motor_para), (320, 23), cv2.FONT_ITALIC, 0.5, (0, 255, 255), 1)
		cv2.arrowedLine(frame, (320, 60), (320 - int(motor_para), 60), (0, 255, 0), 4, line_type = cv2.LINE_AA,
		                tipLength = 0.2)
		L_motor_para = 100 - motor_para / 4
		R_motor_para = 100 + motor_para / 4
		cv2.putText(frame, "LEFT MOTOR:" + str(L_motor_para), (30, 400), cv2.FONT_ITALIC, 0.4, (0, 255, 0), 1)
		cv2.putText(frame, "RIGHT MOTOR:" + str(L_motor_para), (530, 400), cv2.FONT_ITALIC, 0.4, (0, 255, 0), 1)
		cv2.rectangle(frame, (60, 390), (100, 390 - int(L_motor_para)), (0, 255, 255), -1)
		cv2.rectangle(frame, (540, 390), (580, 390 - int(R_motor_para)), (0, 255, 255), -1)
		
	cv2.imshow("camera", frame)
	# cv2.imshow("mask", binary)
	if cv2.waitKey(10) == ord("q"):
		print("System quit.")
		exit_cam(cam)
		exit(0)
