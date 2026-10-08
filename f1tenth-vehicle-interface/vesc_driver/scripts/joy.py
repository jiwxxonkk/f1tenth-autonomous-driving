#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import time

class JoystickVehicleControl:
    def __init__(self):
        rospy.init_node("joystick_vehicle_control", anonymous=True)

        # ROS Subscribers & Publishers
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.drive_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)

        # Parameters
        self.max_speed = 0.8
        self.max_steering_angle = 1.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.straight_speed = 2.5
        self.button_pressed_time = None
        self.button_duration = 2  # seconds
        self.is_moving_straight = False

        # Timer to repeatedly publish drive command
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_drive_cmd)  # every 100ms

    def joy_callback(self, joy_msg):
        # Joystick axes for speed and steering
        self.speed = joy_msg.axes[1] * self.max_speed
        self.steering_angle = joy_msg.axes[3] / 2.0 * self.max_steering_angle
        
        # If button [2] is pressed (button index 2 is typically the third button)
        if joy_msg.buttons[2] == 1 and not self.is_moving_straight:
            self.is_moving_straight = True
            self.button_pressed_time = rospy.get_time()  # Record the time button was pressed

    def publish_drive_cmd(self, event):
        drive_msg = AckermannDriveStamped()

        # If button [2] was pressed, move straight for 10 seconds at 0.1 m/s
        if self.is_moving_straight:
            elapsed_time = rospy.get_time() - self.button_pressed_time
            if elapsed_time < self.button_duration:
                drive_msg.drive.speed = self.straight_speed
                drive_msg.drive.steering_angle = 0.0  # Straight steering
            else:
                self.is_moving_straight = False  # Stop moving after 10 seconds
                drive_msg.drive.speed = 0.0
                drive_msg.drive.steering_angle = 0.0
        else:
            drive_msg.drive.speed = self.speed
            drive_msg.drive.steering_angle = self.steering_angle

        self.drive_pub.publish(drive_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    joystick_control = JoystickVehicleControl()
    joystick_control.run()

