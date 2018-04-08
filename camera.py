#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import RPi.GPIO as GPIO
import time
from os import popen

#舵机引脚定义
FrontServoPin = 23
ServoUpDownPin = 9
ServoLeftRightPin = 11


#设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

#忽略警告信息
GPIO.setwarnings(False)

#舵机引脚初始化
GPIO.setup(FrontServoPin, GPIO.OUT)
GPIO.setup(ServoUpDownPin, GPIO.OUT)
GPIO.setup(ServoLeftRightPin, GPIO.OUT)

# 设置舵机的频率和起始占空比
pwm_FrontServo = GPIO.PWM(FrontServoPin, 50)
pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
pwm_FrontServo.start(0)
pwm_UpDownServo.start(0)
pwm_LeftRightServo.start(0)


class camere(object):
    def __init__(self):
        self.servoUD = 70
        self.servoLR = 70
        self.ServoUpDownPos = self.servoUD
        self.ServoLeftRightPos = self.servoLR
    #前舵机旋转到指定角度
    def frontservo_appointed_detection(self, pos):
        pwm_FrontServo.ChangeDutyCycle(2.5 + 8 * pos / 180)
        time.sleep(0.02)
        # pwm_FrontServo.ChangeDutyCycle(0)	#归零信号

    #摄像头舵机左右旋转到指定角度
    def leftrightservo_appointed_detection(self, pos):
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)
        # pwm_LeftRightServo.ChangeDutyCycle(0)	#归零信号

    #摄像头舵机上下旋转到指定角度
    def updownservo_appointed_detection(self,pos):
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 8 * pos / 180)
        time.sleep(0.02)
        # pwm_UpDownServo.ChangeDutyCycle(0)	#归零信号


    #摄像头舵机向上运动
    def servo_up(self):
        while self.ServoUpDownPos <= 180:
            self.updownservo_appointed_detection(self.ServoUpDownPos)
            time.sleep(0.001)
            self.ServoUpDownPos += 0.7
        if self.ServoUpDownPos >= 180:
            self.ServoUpDownPos = 180


    # 摄像头舵机向下运动
    def servo_down(self):
        while self.ServoUpDownPos >= 35:
            self.updownservo_appointed_detection(self.ServoUpDownPos)
            time.sleep(0.001)
            self.ServoUpDownPos -= 0.7
        if self.ServoUpDownPos <= 35:
            self.ServoUpDownPos = 35


    # 摄像头舵机向左运动
    def servo_left(self):
        while self.ServoLeftRightPos <= 180:
            self.leftrightservo_appointed_detection(self.ServoLeftRightPos)
            time.sleep(0.001)
            self.ServoLeftRightPos += 0.7
        if self.ServoLeftRightPos >= 180:
            self.ServoLeftRightPos = 180


    # 摄像头舵机向右运动
    def servo_right(self):
        while self.ServoLeftRightPos >= 0:
            self.leftrightservo_appointed_detection(self.ServoLeftRightPos)
            time.sleep(0.001)
            self.ServoLeftRightPos -= 0.7
        if self.ServoLeftRightPos <= 0:
            self.ServoLeftRightPos = 0


    # 前舵机向左
    def front_servo_left(self):
        self.frontservo_appointed_detection(180)


    # 前舵机向右
    def front_servo_right(self):
        self.frontservo_appointed_detection(0)

    # 所有舵机归位
    def servo_init(self):
        # servoflag = 0
        servoinitpos = 90
        # if servoflag != servoinitpos:
        self.frontservo_appointed_detection(servoinitpos)
        self.updownservo_appointed_detection(self.servoUD)
        self.leftrightservo_appointed_detection(self.servoLR)
        time.sleep(0.5)
        pwm_FrontServo.ChangeDutyCycle(0)  # 归零信号
        pwm_LeftRightServo.ChangeDutyCycle(0)  # 归零信号
        pwm_UpDownServo.ChangeDutyCycle(0)  # 归零信号


    # 摄像头舵机上下归位
    def servo_updown_init(self):
        self.updownservo_appointed_detection(self.servoUD)


    # 摄像头舵机左右归位
    def servo_leftright_init(self):
        self.leftrightservo_appointed_detection(self.servoLR)


    # 舵机停止
    def servo_stop(self):
        pwm_LeftRightServo.ChangeDutyCycle(0)  # 归零信号
        pwm_UpDownServo.ChangeDutyCycle(0)  # 归零信号
        pwm_FrontServo.ChangeDutyCycle(0)  # 归零信号


    def runCamera(self):
        popen('/home/mjpg-streamer-master/mjpg-streamer-experimental/mjpg_streamer -i "/home/mjpg-streamer-master/mjpg-streamer-experimental/input_uvc.so" -o "/home/mjpg-streamer-master/mjpg-streamer-experimental/output_http.so -w /home/mjpg-streamer-master/mjpg-streamer-experimental/www"')
        self.servo_init()
        time.sleep(5)
        self.servo_down()
        self.servo_up()
        self.servo_updown_init()
        time.sleep(5)
        self.servo_left()
        self.servo_right()
        self.servo_leftright_init()
        time.sleep(5)
        # self.servo_init()
        self.servo_stop()


if __name__ == '__main__':
    camere().runCamera()
    GPIO.cleanup()
