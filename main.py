#!/usr/bin/env python3

__author__ = "Yxzh"

import configparser
import cv2
import numpy as np
import math
import RPi.GPIO as GPIO
import Adafruit_PCA9685 as PCA
import time


# ====== SET VARS ======
kp = 0.25  # PID parameters
ki = 0.01
kd = 0.08
pid_target = 0
max_history_bias = 50
pos = 0  # target bias to frame center
dis = 0  # target distance from camera
motor_para = 0  # motor parameters
L_motor_para = 0
R_motor_para = 0
rush_time = 0.3
loss_target_count = 0
target_loss_frame = 5
is_target = False
SOUND_PIN = 12  # raspberry GPIO pin define
M1A = 35
M1B = 37
M2A = 36
M2B = 38
GPIO.setmode(GPIO.BOARD)  # raspberry GPIO init
GPIO.setup(M1A, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(M1B, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(M2A, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(M2B, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(SOUND_PIN, GPIO.OUT, initial = GPIO.LOW)
motor_PWM_freq = 500  # PWM frequency
servo_PWM_freq = 50
L_A = GPIO.PWM(M1A, motor_PWM_freq)  # start PWM
L_B = GPIO.PWM(M1B, motor_PWM_freq)
R_A = GPIO.PWM(M2A, motor_PWM_freq)
R_B = GPIO.PWM(M2B, motor_PWM_freq)
L_A.start(0)
L_B.start(0)
R_A.start(0)
R_B.start(0)
pwm = PCA.PCA9685(0x40)  # I2C PWM IO for servo
pwm.set_pwm_freq(servo_PWM_freq)

# ====== FUNCTION DEFINE ======
# Camera func
# Open camera and show window
def open_camera():
	camera = cv2.VideoCapture(0)
	camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
	if not camera.isOpened():
		exit_cam(camera)
		print("Unable to initialize camera.")
		exit(1)
	
	# show window
	cv2.namedWindow("camera", 0)
	cv2.resizeWindow("camera", 640, 480)
	cv2.moveWindow("camera", 0, 0)
	return camera

# show window
def show_window(img):
	cv2.imshow("camera", img)

# Exit camera
def exit_cam(cam):
	cam.release()
	cv2.destroyAllWindows()

# Mouse func
# TODO 鼠标点击选择

# Read data func
# Read HSV range from json
def read_color_rangeHSV(filepath, color):
	config = configparser.ConfigParser()
	config.read(filepath, encoding = "utf-8")
	ball_color_range = np.array(config.items(color))
	upper_range = ball_color_range[:3, 1].astype(np.int32).reshape(1, 3)
	lower_range = ball_color_range[3:, 1].astype(np.int32).reshape(1, 3)
	config.clear()
	return upper_range, lower_range

# Write HSV range to json
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
	
# Servo func
# set servo to any angle
def set_servo_angle(chn, angle):
	DualD = int(4096 * (angle * 11 + 500) / 20000 + 0.5)  # convert degree to binary data
	pwm.set_pwm(chn, 0, DualD)

# down arm position
def arm_down():
	set_servo_angle(0, 14)

# default arm position
def arm_up():
	set_servo_angle(0, 31)

# catch
def arm_catch():
	arm_down()
	time.sleep(1.5)
	arm_up()

# Motor func
# break
def motor_break():
	L_A.ChangeDutyCycle(0)
	R_A.ChangeDutyCycle(0)

# slip
def motor_coast():
	GPIO.output(M1A, 1)
	GPIO.output(M1B, 1)
	GPIO.output(M2A, 1)
	GPIO.output(M2B, 1)

# drive motor (range: 0-180)
def motor_drive(L, R):
	GPIO.output(M1A, 1)
	GPIO.output(M1B, 1)
	GPIO.output(M2A, 1)
	GPIO.output(M2B, 1)
	L = L / 180 * 100 + 10
	R = R / 180 * 100
	if L > 100:
		L = 99.99
	if L < 0:
		L = 0.01
	if R > 100:
		R = 99.99
	if R < 0:
		R = 0.01
	print(L, R)
	L_A.ChangeDutyCycle(L)
	R_A.ChangeDutyCycle(R)

# cleanup GPIO pin define
def stop_cleanup():
	L_A.stop()
	L_B.stop()
	R_A.stop()
	R_B.stop()
	GPIO.cleanup()

# Sound func
def beep(f, t):
	f = 1 / f
	for i in range(0, t):
		time.sleep(f)
		GPIO.output(SOUND_PIN, 1)
		time.sleep(f)
		GPIO.output(SOUND_PIN, 0)

# ====== PID CONTROLLER ======
class PID:
	def __init__(self, P, I, D, target, max_history_bias):
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.target = target
		self.bias = [0, 0]
		self.max_history_bias = max_history_bias
	
	# reset all bias
	def clear(self):
		self.bias = [0, 0]
	
	# each control step
	def update(self, feedback):
		self.bias.append(self.target - feedback)
		if len(self.bias) > self.max_history_bias:
			self.bias.pop(0)
		return self.Kp * (self.bias[-1]) + self.Ki * (sum(self.bias)) + self.Kd * (self.bias[-1] - self.bias[-2])

# ====== MAIN PROGRAM ======
# init
cam = open_camera()
arm_up()
controller = PID(kp, ki, kd, pid_target, max_history_bias)

# startup beep
for i in range(300, 600):
	beep(i * 10, int(i / 150))

# main loop
while True:
	ret, frame = cam.read()
	
	# if can'r read from camera
	if not ret:
		exit_cam(cam)
		print("Unable to read frame. (Maybe the video is over?)")
		exit(0)
	
	# image process
	frame_Blur = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	upper, lower = read_color_rangeHSV("./config.ini", "redBall")
	mask = cv2.inRange(frame_Blur, lower, upper)
	mask = cv2.medianBlur(mask, 9)
	mask = cv2.erode(mask, None, iterations = 4)
	mask = cv2.dilate(mask, None, iterations = 13)
	ret, binary = cv2.threshold(mask, 15, 255, cv2.THRESH_BINARY)
	contours, hierarchy = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	
	# if target detected
	if len(contours) > 0:
		
		# refresh PID controller
		if not is_target:
			is_target = True
			controller.clear()
		loss_target_count = 0
		
		# each contours
		for i in range(0, len(contours)):
			x, y, w, h = cv2.boundingRect(contours[i])
			if w * h > 300 and hierarchy[0][i][2] == -1:
				current_father = hierarchy[0][i][2]
				moment = cv2.moments(contours[i])
				center = (
				int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"]))  # center point of mask
				
				# draw target information
				dis = 10 / math.sqrt(w * h)
				pos = center[0] - 320
				cv2.putText(frame, "pos: %d" % (center[0] - 320), (x, y - 5), cv2.FONT_ITALIC, 0.3, (0, 255, 0), 1)
				cv2.putText(frame, "dis: %.3f" % dis, (x, y - 17), cv2.FONT_ITALIC, 0.3, (0, 255, 0), 1)
				cv2.circle(frame, center, 3, (0, 255, 0), 2)
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
				cv2.line(frame, center, (320, center[1]), (255, 0, 0), 1)
	
	# if loss target
	elif is_target:
		loss_target_count += 1
		
		# lost target more than count, then catch
		if loss_target_count > target_loss_frame:
			loss_target_count = 0
			motor_drive(100, 100)
			time.sleep(rush_time)
			motor_break()
			arm_catch()
			is_target = False
			
			# catch beep
			for i in range(600, 680):
				beep(i * 10, int(i / 200))
			for i in range(600, 680):
				beep(i * 10, int(i / 200))
	
	# TODO: 如果从下面出去才抓，单纯丢失目标不抓
	
	# draw screen
	cv2.line(frame, (320, 0), (320, 1000), (0, 0, 0), 1)  # center line
	cv2.putText(frame, "Range: ", (3, 10), cv2.FONT_ITALIC, 0.4, (0, 0, 0), 1)  # HSV range
	cv2.putText(frame, "Upper : " + str(upper[0][0]) + " " + str(upper[0][1]) + " " + str(upper[0][2]), (15, 22),
	            cv2.FONT_ITALIC, 0.4, (0, 0, 0), 1)
	cv2.putText(frame, "Lower : " + str(lower[0][0]) + " " + str(lower[0][1]) + " " + str(lower[0][2]), (15, 34),
	            cv2.FONT_ITALIC, 0.4, (0, 0, 0), 1)
	
	# drive motor and draw screen
	if is_target:
		# drive motor
		motor_para = controller.update(pos)
		L_motor_para = 100 - motor_para / 3
		R_motor_para = 100 + motor_para / 3
		
		# draw information to screen
		motor_drive(L_motor_para, R_motor_para)
		cv2.putText(frame, "PID out: " + str(motor_para), (180, 23), cv2.FONT_ITALIC, 0.8, (0, 255, 255), 2)
		cv2.arrowedLine(frame, (320, 60), (320 + int(motor_para), 60), (0, 255, 0), 2, line_type = cv2.LINE_AA,
		                tipLength = 0.2)
		cv2.putText(frame, "LEFT MOTOR:" + str(L_motor_para), (30, 400), cv2.FONT_ITALIC, 0.4, (0, 255, 0), 1)
		cv2.putText(frame, "RIGHT MOTOR:" + str(R_motor_para), (430, 400), cv2.FONT_ITALIC, 0.4, (0, 255, 0), 1)
		cv2.rectangle(frame, (60, 390), (100, 390 - int(L_motor_para)), (0, 255, 255), -1)
		cv2.rectangle(frame, (540, 390), (580, 390 - int(R_motor_para)), (0, 255, 255), -1)
	
	# show screen
	show_window(frame)
	# 	# cv2.imshow("mask", binary)
	
	# if quit
	if cv2.waitKey(10) == ord("q"):
		print("System quit.")
		exit_cam(cam)
		stop_cleanup()
		exit(0)
