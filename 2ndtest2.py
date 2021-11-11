#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import time
import math
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

Freq = 100000  # Hzを上げると音が聞きづらくなるが、熱を持つ
base_duty = 70

dst_max = 300
dst_level = 0
dst_ratio = {0,0.2,0.4,0.6,0.8,1}

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

def dutyToPer(duty):
    return int(duty * 1000000 / 100.)

class Motor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.speed = 0
        self.ang = 0
        self.delta = 0
        self.kill = False

    def run(self):
        while not self.kill:
            # print(self.ang)
            # print(self.speed)
            if self.speed <= 0:
                #print("Speed Low")
                pi.write(SWpin,0)
            else:
                #print("Motor On")
                pi.write(SWpin,1)
                if self.ang > 0: #左寄り
                    pi.hardware_PWM(gpio_pinR, Freq, dutyToPer(((base_duty*self.speed)+self.delta)*dst_ratio[dst_level]))
                    pi.hardware_PWM(gpio_pinL, Freq, dutyToPer(((base_duty*1.1*self.speed)-self.delta)*dst_ratio[dst_level]))
                    print("Lside")
                elif self.ang < 0: #右寄り
                    pi.hardware_PWM(gpio_pinR, Freq, dutyToPer(((base_duty*self.speed)+self.delta)*dst_ratio[dst_level]))
                    pi.hardware_PWM(gpio_pinL, Freq, dutyToPer(((base_duty*1.1*self.speed)-self.delta)*dst_ratio[dst_level]))
                    print("Rside")
                    
                else:    
                    pi.hardware_PWM(gpio_pinR, Freq, dutyToPer((base_duty*self.speed)*dst_ratio[dst_level]))
                    pi.hardware_PWM(gpio_pinL, Freq, dutyToPer((base_duty*1.1*self.speed)*dst_ratio[dst_level]))
            # print(self.delta)
            time.sleep(0.1)

class Ultrasonic(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.kill = False

    def read_distance():
        pi.write(TRIGpin, 1)
        time.sleep(0.00001)
        pi.write(TRIGpin, 0)

        while pi.read(ECHOpin) == 0:
            sig_off = time.time()
        while pi.read(ECHOpin) == 1:
            sig_on = time.time()

        duration = sig_off - sig_on
        distance = duration * 34000 / 2.0
        return distance

    def run(self):
        while not self.kill:
            dst = self.read_distance()
            if dst < dst_max:
                dst_level = round(dst/50)

class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ang = 0
        self.delta = 0
        rospy.init_node('Car')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__callback)
        # self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.__callback)

    def __callback(self, raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # self.twist = {"speed": raw.linear.x, "ang": raw.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        rospy.logdebug("Autoware > %s" % self.twist)
        self.ang = twist["ang"]
        self.speed = twist["speed"]
        self.delta = math.asin(self.ang)

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        self.subscriber.unregister()

    def get_Twist(self):
        return self.speed, self.ang, self.delta

class Joystick:
    def __init__(self):
        self.select_button = 0
        self.ciurcle = 0
        self.cross = 0
        self.speed = 0
        self.ang = 0
        self.delta = 0
        self.subscriber = rospy.Subscriber('/joy', Joy, self.__callback)

    def __callback(self, raw):
        self.select_button = raw.buttons[10]
        self.ciurcle = raw.buttons[3]
        self.cross = raw.buttons[2]
        self.speed = raw.axes[3]
        self.ang = raw.axes[0]
        self.delta = math.asin(self.ang)-math.atan(self.ang)
        

    def get_button(self):
        return self.select_button, self.ciurcle, self.cross
    
    def get_Twist(self):
        return self.speed, self.ang, self.delta

if __name__ == '__main__':
    print("In The Main Function!")
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
                m.delta = status[2]
            else:
                status = a.get_Twist()
                m.speed = status[0]
                m.ang = status[1]
                m.delta = status[2]
            rospy.sleep(0.01)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    m.kill = True
    terminate()