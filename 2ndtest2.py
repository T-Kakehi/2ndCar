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

gpio_pinR = 13
gpio_pinL = 12
DIRpin = 6
SWpin = 15

TRIGpin = 22
ECHOpin = 23

Rmotor_ini = 0.95
Lmotor_ini = 1
T = 0.475 # 車輪幅0.475[m]

Freq = 100000  # Hzを上げると音が聞きづらくなるが、熱を持つ
base_duty = 70

history = collections.deque(maxlen=10)
dst_min = 5
dst_max = 300
dst_ratio = [0,0.2,0.4,0.6,0.8,1]

pi = pigpio.pi()
pi.set_mode(gpio_pinR, pigpio.OUTPUT)
pi.set_mode(gpio_pinL, pigpio.OUTPUT)
pi.set_mode(DIRpin, pigpio.OUTPUT)
pi.set_mode(SWpin,pigpio.OUTPUT)

pi.set_mode(TRIGpin,pigpio.OUTPUT)
pi.set_mode(ECHOpin,pigpio.INPUT)

print("Pin Setup Completed!")

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
    return int(duty * 1000000 / 100.)

def culc_power(v, omg):
    inverse_mat = np.linalg.pinv(np.array([[1/2,1/2],[1/T,-1/T]]))
    mat = np.array([[v],[omg]])
    return inverse_mat.dot(mat)

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
        while pi.read(ECHOpin) == 1:
            StopTime = time.time()
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2
        return distance

    def distance_filtered(self):
        history.append(self.read_distance())
        return np.median(history)

    def run(self):
        while not self.kill:
            dst = self.distance_filtered()
            print("---dst---")
            print(dst)
            time.sleep(0.2)

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

    def run(self):
        while not self.kill:
            # print(self.ang)
            # print(self.speed)
            if self.speed <= 0:
                #print("Motor Stop")
                pi.write(SWpin,0)
            else:
                #print("Motor On")
                pi.write(SWpin,1)
                if self.ang > 0: #左寄り
                    pi.hardware_PWM(gpio_pinR, Freq, duty2per(base_duty*Rmotor_ini*self.Rpower*dst_ratio[us.get_level()]))
                    pi.hardware_PWM(gpio_pinL, Freq, duty2per(base_duty*Lmotor_ini*self.Lpower*dst_ratio[us.get_level()]))
                    print("Lside")
                elif self.ang < 0: #右寄り
                    pi.hardware_PWM(gpio_pinR, Freq, duty2per(base_duty*Rmotor_ini*self.Rpower*dst_ratio[us.get_level()]))
                    pi.hardware_PWM(gpio_pinL, Freq, duty2per(base_duty*Lmotor_ini*self.Lpower*dst_ratio[us.get_level()]))
                    print("Rside")
                    
                else:    
                    pi.hardware_PWM(gpio_pinR, Freq, duty2per(base_duty*Rmotor_ini*self.speed*dst_ratio[us.get_level()]))
                    pi.hardware_PWM(gpio_pinL, Freq, duty2per(base_duty*Lmotor_ini*self.speed*dst_ratio[us.get_level()]))
            # print(self.delta)
                print(self.Rpower)
                print(self.Lpower)
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
        # self.twist = {"speed": raw.linear.x, "ang": raw.angular.z}  # speed: m/s, angular: radian/s
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
        self.ciurcle = 0
        self.cross = 0
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        self.subscriber = rospy.Subscriber('/joy', Joy, self.__callback)

    def __callback(self, raw):
        self.select_button = raw.buttons[10]
        self.ciurcle = raw.buttons[3]
        self.cross = raw.buttons[2]
        self.speed = raw.axes[3]
        self.ang = raw.axes[0]
        power = culc_power(self.speed, self.ang)
        self.Rpower = sigmoid_func(power[0][0])
        self.Lpower = sigmoid_func(power[1][0])

    def get_button(self):
        return self.select_button, self.ciurcle, self.cross
    
    def get_Twist(self):
        return self.speed, self.ang, self.Rpower, self.Lpower

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
        while not rospy.is_shutdown():
            buttons = j.get_button()
            if (buttons[0] and buttons[2]):
                if joy_flag:
                    print("---Out joy mode---")
                    joy_flag = False
            elif (buttons[0] and buttons[1]) or joy_flag:
                if not joy_flag:
                    print("---In joy mode---\n[INFO]\nPlease check a centre light red blink")
                    joy_flag = True
                status = j.get_Twist()
                m.speed = status[0]
                m.ang = status[1]
                m.Rpower = status[2]
                m.Lpower = status[3]
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