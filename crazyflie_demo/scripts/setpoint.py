#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from crazyflie.msg import Full
from scipy import io
import os

class TrajectoryCircle:
    def __init__(self):
        self.r = 0.0
        self.h = 0.0
        self.t = 0.0
        self.rt = 50.0
        self.gr = 0
        self.button_pressed = False
        self.start_time = rospy.Time()

        self.pub = rospy.Publisher('cmd_full', Full, queue_size=1)
        self.start_pub = rospy.Publisher('/start', Twist, queue_size=1)

        self.xi = rospy.get_param("~xi", 0.0)
        self.yi = rospy.get_param("~yi", 0.0)
        self.zi = rospy.get_param("~zi", 0.0)
        self.altura = rospy.get_param("~h", 0.0)
        self.radio = rospy.get_param("~r", 0.0)

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[2] == 1 and not self.button_pressed:
            rospy.loginfo('Trayectoria iniciada')
            self.button_pressed = True
            self.r = self.radio
            self.h = self.altura
            self.t = 0.0
            self.gr = 10
            self.start_time = rospy.Time.now()

        elif joy_msg.buttons[10] == 1 and self.button_pressed:
            rospy.loginfo('Trayectoria reiniciada')
            self.r = 0.0
            self.h = 0.0
            self.t = 0.0
            self.gr = 0
            self.button_pressed = False
            self.start_time = rospy.Time()  # Reset the starting time
            start_msg = Twist()
            start_msg.angular.x = 0
            self.start_pub.publish(start_msg)

    def trajectory_circle(self):
        rospy.sleep(1)  # Wait 1 second before starting the trajectory
        rate = rospy.Rate(self.rt)  # rt Hz

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if self.button_pressed:  # Verificar si la trayectoria ha sido iniciada
                if not self.start_time.is_zero():
                    elapsed_time = (current_time - self.start_time).to_sec()
                    # rospy.loginfo('Tiempo: ' + str(elapsed_time))
                    if elapsed_time >= 60:
                        start_msg = Twist()
                        start_msg.angular.x = 0
                        self.start_pub.publish(start_msg)
                        rospy.loginfo('Trayectoria Terminada')
                        break

                full_msg = Full()
                full_msg.header = Header()
                full_msg.header.stamp = rospy.Time.now()

                self.t = elapsed_time
                w = np.pi / 6
                p = 15

                x = self.r * (np.arctan(p) + np.arctan(self.t - p)) * np.cos(w * self.t) + self.xi
                y = self.r * (np.arctan(p) + np.arctan(self.t - p)) * np.sin(w * self.t) + self.yi
                z = (self.h / 2) * (1 + np.tanh(self.t - 2.5)) + self.zi
                yaw = 0

                dx = -self.r * (np.arctan(p) + np.arctan(self.t - p)) * w * np.sin(w * self.t)
                dy = self.r * (np.arctan(p) + np.arctan(self.t - p)) * w * np.cos(w * self.t)
                dz = (self.h / 2) * np.tanh(self.t - 2.5) * (1 - np.tanh(self.t - 2.5))
                dyaw = 0

                ddx = -self.r * (np.arctan(p) + np.arctan(self.t - p)) * w**2 * np.cos(w * self.t)
                ddy = -self.r * (np.arctan(p) + np.arctan(self.t - p)) * w**2 * np.sin(w * self.t)
                ddz = (self.h / 2) * np.tanh(self.t - 2.5) * (1 - np.tanh(self.t - 2.5)) * (1 - 2 * np.tanh(self.t - 2.5))
                ddyaw = 0

                full_msg.twist1 = Twist()
                full_msg.twist1.linear.x = x
                full_msg.twist1.linear.y = y
                full_msg.twist1.linear.z = z
                full_msg.twist1.angular.x = dx
                full_msg.twist1.angular.y = dy
                full_msg.twist1.angular.z = dz

                full_msg.twist2 = Twist()
                full_msg.twist2.linear.x = ddx
                full_msg.twist2.linear.y = ddy
                full_msg.twist2.linear.z = ddz
                full_msg.twist2.angular.x = yaw
                full_msg.twist2.angular.y = dyaw
                full_msg.twist2.angular.z = ddyaw

                start_msg = Twist()
                start_msg.angular.x = 10
                self.start_pub.publish(start_msg)

                self.pub.publish(full_msg)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('trajectory_circle', anonymous=True)
    traj_circle = TrajectoryCircle()
    joy_sub = rospy.Subscriber('joy', Joy, traj_circle.joy_callback)
    traj_circle.trajectory_circle()
    rospy.spin()
