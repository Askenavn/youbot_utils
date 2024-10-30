#!/usr/bin/env python3
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import pygame
import time
import rospy, math


def open_Gripper():
    data = Float64MultiArray()
    grip_topic = rospy.Publisher('/youbot_arm/gripper_pos_controller/command',
                                 Float64MultiArray, queue_size=1)
    data.data = [0.01, 0.01]
    grip_topic.publish(data)
    print("open gripper")

def close_Gripper():
    data = Float64MultiArray()
    grip_topic = rospy.Publisher('/youbot_arm/gripper_pos_controller/command',
                                 Float64MultiArray, queue_size=1)
    data.data = [0.0, 0.0]
    grip_topic.publish(data)
    print("close gripper")

def arm_pos_cmd(q1 = 0.0101, q2 = 0.0101, q3 = -0.016, q4 = 0.023, q5 = 0.12):
        msg = Float64MultiArray()
        msg.data = [q1, q2, q3, q4, q5]
        arm_topic.publish(msg)


pygame.init()

cnst_vel = 0.25
joystick = None
run = False


if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("joystick initialized")
    rospy.init_node('gamepad_controll')
    pub_vel = rospy.Publisher('/youbot_base/mecanum_drive_controller/cmd_vel',
                               Twist, queue_size=10)
    arm_topic = rospy.Publisher('/youbot_arm/joints_pos_controller/command',
                                Float64MultiArray, queue_size=1)
    run = True
    mode = 'first_part'
    set_vel = Twist()
    time.sleep(0.5)


while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            print(pygame.key.name(event.key))


    if joystick:
        ##################################################
        
        #Manipulator

        if joystick.get_button(0):
            arm_pos_cmd()
            print('A - Home position')

        if joystick.get_button(6):
            mode = 'second_part'
            time.sleep(0.1)
            print(mode)

        elif joystick.get_button(7):
            mode = 'first_part'
            time.sleep(0.1)
            print(mode)

        if mode == 'first_part':
            
            if joystick.get_button(1):
                print('B - Помахать')

                dq = 0.8
                q1 = 2.9
                q2 = 1.05
                q3 = -2.43-dq/2
                q4 = 1.7-dq/2
                # q5 = 3

                # q5 = 1.7
                arm_pos_cmd(q1, q2, q3, q4)
                time.sleep(0.25)

                for i in range(6):
                    # q1 += 0.1*(-1)**i
                    q2 = q2
                    q3 += dq*(-1)**i
                    q4 += dq*1.5*(-1)**i
                    time.sleep(1.2)
                    arm_pos_cmd(q1, q2, q3, q4)
                    
                arm_pos_cmd(2.9, 1.05, -2.43, 1.7)

            elif joystick.get_button(2):
                print("X - take or put")
                # arm_pos_cmd()
                # time.sleep(3.5)
                arm_pos_cmd(2.9, 1.05, -2.43, 1.7)
                time.sleep(2.5)
                arm_pos_cmd(2.9, 1.7, -1.5, 3.0)
                time.sleep(0.8)
                arm_pos_cmd(2.9, 2.3, -1.5, 2.8)
                time.sleep(0.5)
            
            elif joystick.get_button(3):
                arm_pos_cmd(2.9, 1.05, -2.43, 1.7)
                print("Y - Arm up")

        elif mode == 'second_part':
            if joystick.get_button(1):
                print("B - Goose")
                arm_pos_cmd(2.9, 1.05, -2.43, 3.)

            elif joystick.get_button(2):
                print("X - No")
                arm_pos_cmd(2.9, 1.05, -2.43, 3.)
                for i in range(3):
                    q1 = 2.9 + 0.5 * (-1)**i
                    arm_pos_cmd(q1, 1.05, -2.43, 3.14)
                    rospy.sleep(1.2)
                arm_pos_cmd(2.9, 1.05, -2.43, 3.)

            elif joystick.get_button(3):
                print("Y - Yes")
                arm_pos_cmd(2.9, 1.05, -2.43, 3.)
                for i in range(4):
                    q4 = 3. + 0.3 * (-1)**i
                    arm_pos_cmd(2.9, 1.05, -2.43, q4)
                    rospy.sleep(0.8)
                arm_pos_cmd(2.9, 1.05, -2.43, 3.)

        #EndManipulator
        ##################################################
        
        #Gripper
        if joystick.get_button(4):
            open_Gripper()
        elif joystick.get_button(5):
            close_Gripper()
        #EndGripper
        ##################################################

        #Base
        # if mode == 'first_part':
        dir = joystick.get_hat(0)
        # y = dir[0]
        # x = dir[1]

        # elif mode == 'second_part':
        y = round(joystick.get_axis(0)**3, 2) + dir[0]
        x = round((joystick.get_axis(1))**3, 2) - dir[1]
        set_vel.linear.x = x * cnst_vel
        set_vel.linear.y = y * cnst_vel

        z = round(-joystick.get_axis(3)**3, 1)
        set_vel.angular.z = z * cnst_vel*1.5

        pub_vel.publish(set_vel)
        print(set_vel.linear.x, set_vel.linear.y, set_vel.angular.z)
        print(cnst_vel)

        #EndBase
        ##################################################

        if joystick.get_axis(2) < 0:
            cnst_vel += 0.005
            time.sleep(0.05)

        if joystick.get_axis(5) < 0:
            cnst_vel -= 0.005
            time.sleep(0.05)

        # print(joystick.get_axis(2))
        # print(joystick.get_axis(5))


        if joystick.get_button(8):
            run = False
            close_Gripper()

    # time.sleep(0.1)


pygame.quit()
exit()


# 0 - A
# 1 - B
# 2 - Y
# 3 - X
# 4 - leftButt
# 5 - rightButt
# 6 - windows
# 7 - list
# 8 - Xbox
# 9 - leftStick
# 10 - rightStick

# axis0,1 - left
# axis3,4 - right

# hat(0) - (0,0)
