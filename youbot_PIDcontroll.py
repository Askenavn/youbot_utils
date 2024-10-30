#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import rospy, math
from PID import PIDcontroller
from scipy.interpolate import CubicSpline
import numpy as np
from matplotlib import pyplot as plt


class Youbot:
    def __init__(self, robotID: str):
        rospy.init_node('youbot_controll')
        self.robotID = robotID
        self.cur_pose = PointStamped()
        self.pub_cmd_vel = rospy.Publisher('/base/cmd_vel', Twist, queue_size=1)
        self.pid = PIDcontroller()

        #velocity params
        self.velMax = 0.3
        self.velMin = 0.02
        self.cur_vel = 0.0

    def update_pose(self):
        rospy.Subscriber('/marker_pose', PointStamped, self.callback_pose)


    def callback_pose(self, data: PointStamped):
        rospy.Subscriber('/odom', Odometry, self.callback_odom) 
        if data.header.frame_id == self.robotID:
            self.cur_pose = data

    def callback_odom(self, data: Odometry):

    

    def set_goal(self, goal_x, goal_y, threshold=0.005):
        vel = Twist()
        coef_P = 0.1

        error_x = goal_x - self.cur_pose.point.x
        while abs(error_x) > abs(threshold):
            vel.linear.x = coef_P * error_x
            error_x = goal_x - self.cur_pose.point.x
            self.pub_cmd_vel.publish(vel)

        # error_y = goal_y - self.cur_pose.point.y
        # while abs(error_y) > abs(threshold):
        #     vel.linear.y = coef_P * error_y
        #     error_y = goal_y - self.cur_pose.point.y
        #     self.pub_cmd_vel.publish(vel)

        self.pub_cmd_vel.publish(Twist())

    def set_goal_with_pid(self, goal_x: float, goal_y: float):
        '''Calculate speeds, time for robot using PID-controller \n
        goal_x, goal_y: in meters'''
        vel = Twist()

        vectors = self.vector_to_goal(goal_x, goal_y)
        goal_vel = self.fuzzy(vectors)
        self.cur_vel, _ = self.pid.update(goal_vel, self.cur_vel) #absolute speed
        # dec_way = self.decseleration_way() #distance to stop moving

        while vectors[2] >= 0.009:
            try:
                tau = vectors[2] / self.cur_vel #time for calculation speeds
            except ZeroDivisionError:
                tau = 100000000
            v_x = vectors[0] / tau #speed in x-axis
            v_y = vectors[1] / tau #speed in y-axis
            vel.linear.x = v_x
            vel.linear.y = v_y
            self.pub_cmd_vel.publish(vel)

            # if vectors[2] <= dec_way:
            #     abs_v, _ = self.pid.update(self.velMin, abs_v)
            # else:
            #     abs_v, _ = self.pid.update(self.velMax, abs_v)
            vectors = self.vector_to_goal(goal_x, goal_y)
            goal_vel = self.fuzzy(vectors)
            self.cur_vel, _ = self.pid.update(goal_vel, self.cur_vel)
        return
  
    def decseleration_way(self):
        abstract_pid =  PIDcontroller(self.pid.Kp, self.pid.Ki, self.pid.Kd)
        v, t = abstract_pid.update(0.0, self.velMax)
        while v > self.velMin+0.001:
            v, t = abstract_pid.update(self.velMin, v)
        s = self.velMax * t * 0.5
        print(s)
        return s
            
    def vector_to_goal(self, goal_x, goal_y):
        '''Calculate vector to goal from current position
        \nReturns a tuple of vector_x, vector_y, vector_module'''
        vector_x = goal_x - self.cur_pose.point.x
        vector_y = goal_y - self.cur_pose.point.y
        vector_module = math.sqrt(vector_x**2 + vector_y**2)
        return (vector_x, vector_y, vector_module)
    
    def max_simulation_time(self, vector: tuple):
        '''Calculate max time needed for motion to goal place \n
          vector: (x, y, module) \n
          return max sample time according to min velocity'''
        return vector[2] / self.velMin
    
    def calculate_acselerations(self, vector: tuple, max_dist: float, dt: float):
        '''Calculate speeds for current vector \n
        vector: (x, y, module) \n
        max_dist: distance for acseleration and decseleration \n
        dt: discretisation of time \n'''
        
        '''From S = (v2**2-v1**2)/2a get a'''
        self.a = (self.velMax**2 - self.velMin**2) / (2 * max_dist)

        '''From S = Vt + 0.5at^2 get t for acseleration and decseleration
        0.5 * a * t^2 + V * t - S = 0
        D = V**2 - 0.5*a*S
        t = [-V +- sqrt(D)] / a'''
        if (max_dist >= vector[2] / 2):
            D = self.velMin**2 - 0.5*a*vector[2]/2
        else:
            D = self.velMin**2 - 0.5*a*max_dist
        t1 = [-self.velMin + math.sqrt(D)] / a
        t2 = [-self.velMin - math.sqrt(D)] / a
        t_a = max(t1,t2) #Time for acseleration and decseleration
        
        '''From S = Vmax*t get t for uniform motion'''
        if (max_dist < vector[2] / 2):
            t_m = (vector[2] - (2 * max_dist)) / self.velMax
        else:
            t_m = 0
        
        if (2*t_a + t_m) >= self.max_simulation_time(vector):
            raise TimeoutError('Time for motion out of sample time')
        
        t_a_error = t_a % dt #error of discretisation for ac/decseleration
        n_a = t_a // dt #nums of iteration for ac/decseleration
        t_m_error = t_a % dt #error of discretisation for uniform motion
        n_m = t_m // dt #nums of iteration for uniform motion

        return (n_a, n_m)
        
    def fuzzy(self, vector):
        if vector[2] < 0.2:
            goal_vel = self.velMin
        elif vector[2] >= 0.2 and vector[2] <= 0.4:
            goal_vel = self.velMax / 2
        elif vector[2] > 0.4:
            goal_vel = self.velMax
        return goal_vel
    
    def move_by_spline(self, x, y, dt = 0.2, n=10):
        '''x and y is lists of coordinates by axes'''
        '''dt is discretization of time'''
        t = [dt*i for i in range(len(x))]
        line_x = CubicSpline(t, x, bc_type='natural')
        line_y = CubicSpline(t, y, bc_type='natural')
        t_new = np.linspace(t[0], t[len(t)-1], n)
        x_new = line_x(t_new)
        y_new = line_y(t_new)

        for i in range(len(x_new)):
            print(f'Going to pose: (x: {x_new[i]}, y: {y_new[i]})')
            self.set_goal_with_pid(x_new[i], y_new[i])
            print(self.cur_pose.point)   

        self.pub_cmd_vel.publish(Twist())

        
def move_by_spline1(x, y, dt = 0.2, n = 20):
    '''x and y is lists of coordinates by axes'''
    '''dt is discretization of time'''
    '''n is nums of points'''
    t = [dt*i for i in range(len(x))]
    line_x = CubicSpline(t, x, bc_type='natural')
    line_y = CubicSpline(t, y, bc_type='natural')
    t_new = np.linspace(t[0], t[len(t)-1], n)
    x_new = line_x(t_new)
    y_new = line_y(t_new)
    print(len(x_new))
    plt.figure(figsize = (10,8))
    plt.plot(x_new, y_new, 'b')
    plt.plot(x, y, 'ro')
    plt.title('Cubic Spline Interpolation')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()
   

def callback_odom(data):
    pass


if __name__ == "__main__":
    # # while True:
    #     x = float(input("x: "))
    #     y = float(input("y: "))
    #     u.set_goal_with_pid(x, y)
    #     print(u.cur_pose)
    #     u.pub_cmd_vel.publish(Twist())
    # x = [0.1, 0.2, 0.4, 0.4, 0.5, 0.6, 0.7]
    # y = [0.6, 0.6, 0.0, 0.6, 0.0, 0.6, 0.0]
    # move_by_spline1(x, y, n = len(x))
    # u = Youbot('\x01')
    # rospy.sleep(0.5)
    # u.move_by_spline(x, y, n = len(x))
