#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import time
import collections
import numpy as np
import rospy
import pigpio
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

gpio_pinR = 12
gpio_pinL = 13
DIRpin = 6
SWpin = 15

TRIGpin = 22
ECHOpin = 23

Rmotor_ini = 1
Lmotor_ini = 1
T = 0.475 # 車輪幅0.475[m]

Freq = 100000  # Hzを上げると音が聞きづらくなるが、熱を持つ
base_duty = 100

sonic_speed = 34300
history = collections.deque(maxlen=10)
dst_min = 50.
dst_max = 250.
dst_gap = 10.
#最大測定距離、最低確保距離、分解能から出力の割合表を作成
elem = (dst_max-dst_min)/dst_gap
print(elem)
dst_ratio = [i/elem for i in range(int(elem+1))]
#最大測定距離から返ってくる音波の最大時間を計算
max_sec = dst_max/sonic_speed*2
print("[INFO]dst_elements\n"+str(dst_ratio))

pi = pigpio.pi()
pi.set_mode(gpio_pinR, pigpio.OUTPUT)
pi.set_mode(gpio_pinL, pigpio.OUTPUT)
pi.set_mode(DIRpin, pigpio.OUTPUT)
pi.set_mode(SWpin,pigpio.OUTPUT)

pi.set_mode(TRIGpin,pigpio.OUTPUT)
pi.set_mode(ECHOpin,pigpio.INPUT)

print("[INFO]\nPin Setup Completed!")

def terminate():
    try:
        pi.write(gpio_pinR, 0)
        pi.write(gpio_pinL, 0)
        pi.write(DIRpin, 0)
        pi.write(SWpin, 0)
    except Exception:
        pass
    finally:
        print("Terminated!")
        pi.stop()

def duty2per(duty):
    return int(duty * 1000000 / 100.) # duty 0~1M

def culc_power(v, omg):
    # a = np.array([[1/2,1/2],[1/T,-1/T]])
    # b = np.array([[v],[omg]])
    # return np.linalg.solve(a,b)
    a = np.array([[0.5,0.5],[1/T,-1/T]])
    # print(a)
    inverse = np.linalg.pinv(a)
    # print(inverse)
    b = np.array([[v],[omg]])
    v = inverse.dot(b)
    # print(v)
    return v

def sigmoid_func(raw):
    return 1/(1+np.e**-raw)

class Ultrasonic(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.kill = False
        self.dst_level = 0

    def read_distance(self):
        pi.write(TRIGpin, 1)
        time.sleep(0.00001)
        pi.write(TRIGpin, 0)
        StartTime = time.time()
        StopTime = time.time()
        while pi.read(ECHOpin) == 0:
            StartTime = time.time()
        while pi.read(ECHOpin) == 1 and(time.time() - StartTime) < max_sec:
            StopTime = time.time()
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * sonic_speed) / 2
        if distance < dst_min:
            distance = dst_min
        # print("---dst---")
        # print(distance)
        return distance

    def distance_filtered(self):
        history.append(self.read_distance())
        return np.median(history)

    def run(self):
        while not self.kill:
            dst = self.distance_filtered()
            #最低限の確保距離からどれだけ距離の余裕があるかを分割した単位あたりの距離で割って比率を出す
            # print("---dst---")
            # print(dst)
            self.dst_level = int((dst-dst_min)/(dst_max-dst_min)*len(dst_ratio))
            if self.dst_level < 0:
                self.dst_level = 0
            # print("---level---")
            # print(dst_ratio[self.dst_level])
            time.sleep(0.5)

    def get_level(self):
        return self.dst_level

class Motor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.kill = False
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        self.Rduty = 0
        self.Lduty = 0

    def set_motor(self, motor):
        self.speed = motor[0]
        self.ang = motor[1]
        self.Rpower = motor[2]
        self.Lpower = motor[3]

    def stop_motor(self):
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0

    def run(self):
        while not self.kill:
            if self.speed <= 0:
                #print("Motor Stop")
                pi.write(SWpin,0)
            else:
                #print("Motor On")
                pi.write(SWpin,1)
                self.Rduty = base_duty*Rmotor_ini*self.Rpower*dst_ratio[us.get_level()]
                self.Lduty = base_duty*Lmotor_ini*self.Lpower*dst_ratio[us.get_level()]
                if self.Rduty > 100:
                    self.Lduty = self.Lduty - (self.Rduty - 100)
                    self.Rduty = 100
                elif self.Rduty < 0:
                    self.Lduty = self.Lduty + (abs(self.Rduty))
                    self.Rduty = 0
                if self.Lduty > 100:
                    self.Rduty = self.Rduty - (self.Lduty - 100)
                    self.Lduty = 100
                elif self.Lduty < 0:
                    self.Rduty = self.Rduty + (abs(self.Lduty))
                    self.Lduty = 0
                pi.hardware_PWM(gpio_pinR, Freq, duty2per(self.Rduty))
                pi.hardware_PWM(gpio_pinL, Freq, duty2per(self.Lduty))
            time.sleep(0.1)

class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        rospy.init_node('Car')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__callback)

    def __callback(self, raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        rospy.logdebug("Autoware > %s" % self.twist)
        self.ang = twist["ang"]
        self.speed = twist["speed"]
        power = culc_power(self.speed, self.ang)
        self.Rpower = power[0][0]
        self.Lpower = power[1][0]

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        self.subscriber.unregister()

    def get_Twist(self):
        return self.speed, self.ang, self.Rpower, self.Lpower

class Joystick:
    def __init__(self):
        self.select_button = 0
        self.start_button = 0
        self.ciurcle = 0
        self.cross = 0
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        self.subscriber = rospy.Subscriber('/joy', Joy, self.__callback)

    def __callback(self, raw):
        self.select_button = raw.buttons[10]
        self.start_button = raw.buttons[11]
        self.ciurcle = raw.buttons[3]
        self.cross = raw.buttons[2]
        self.speed = raw.axes[3]
        self.ang = raw.axes[2]
        power = culc_power(self.speed, self.ang)
        self.Rpower = power[0][0]
        self.Lpower = power[1][0]

    def get_button(self):
        return self.select_button, self.ciurcle, self.cross, self.start_button
    
    def get_Twist(self):
        return self.speed, self.ang, self.Rpower, self.Lpower

class Detect_White:
    def __init__(self):
        self.detect = False
        self.cnt = 0
        self.subscriber = rospy.Subscriber('/detect_whiteline', Bool, self.__callback)
    
    def __callback(self, raw):
        self.detect = raw.data
        if raw.data:
            self.cnt += 1

    def get_detect(self):
        return self.detect

    def get_cnt(self):
        return self.cnt

white = 0
if __name__ == '__main__':
    print("In The Main Function!")
    us = Ultrasonic()
    us.start()
    m = Motor()
    m.start()
    joy_flag = False
    try:
        a = Autoware()
        j = Joystick()
        dw = Detect_White()
        while not rospy.is_shutdown():
            buttons = j.get_button()
            detect = dw.get_detect()
            if detect or white:
                if not white:
                    print("---In white stop---\n[INFO]This is "+str(dw.get_cnt())+"s whiteline!\nPlease put a start button")
                white = 1
                m.stop_motor()
                if buttons[3] == 1:
                    print("Move forward")
                    start = time.time()
                    while (time.time() - start) < 1:
                        m.set_motor([1, 0, 1, 1])
                    white = 0
                    print("---Fin white line---")
            else:
                if (buttons[0] and buttons[2]):
                    if joy_flag:
                        print("---Out joy mode---")
                        joy_flag = False
                elif (buttons[0] and buttons[1]) or joy_flag:
                    if not joy_flag:
                        print("---In joy mode---\n[INFO]\nMake sure it's glowing red.")
                        joy_flag = True
                    m.set_motor(j.get_Twist())
                    # m.speed = status[0]
                    # m.ang = status[1]
                    # m.Rpower = status[2]
                    # m.Lpower = status[3]
                else:
                    status = a.get_Twist()
                    m.speed = status[0]
                    m.ang = status[1]
                    m.Rpower = status[2]
                    m.Lpower = status[3]
            rospy.sleep(0.01)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    m.kill = True
    us.kill = True
    terminate()