#!/usr/bin/env python
from matplotlib import pyplot as plt
import time
from scipy import interpolate

class PIDcontroller:
    def __init__(self, Kp=0.00002, Ki=0.02, Kd=0.00002):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.t0 = time.time()
        
        # self.e_prev = 0.0
        # self.Integrator = 0.0

        # self.dt = 0.001

    def T(self):
        # time.sleep(self.dt)
        t1 = time.time()
        return t1 - self.t0

    def update(self, v_in, v_back):
        e = v_in - v_back
        P = self.Kp * e
        t = self.T()
        I = self.Kp * self.Ki * e * t * 0.5
        D = self.Kp * self.Kd * e / t
        u = v_back + P+I+D
        return u, t

    # def update(self, v_in, v_back):
    #     e = v_in - v_back
    #     P = self.Kp * e
    #     t = self.T()
    #     self.Integrator = self.Integrator + e
    #     I = self.Ki * self.Integrator
    #     D = self.Kd * (e - self.e_prev) / self.dt
    #     u = P+I+D
    #     return u, t

if __name__ == '__main__':
    pid = PIDcontroller()
    v_in = 0.3
    v_start = 0.0
    time.sleep(0.0000001)
    v_back, t = pid.update(v_in, v_start)
    tF = []
    tp = []
    # tF.append(v_back)
    # tp.append(t)
    i = 0
    while v_back < v_in - 0.001:
        v_back, t0 = pid.update(v_in, v_back)
        tF.append(v_back)
        tp.append(t0)
    # print(tp[4]-tp[3])
    # print(tp[10]-tp[9])
    # print(tp[2000]-tp[1999])
    # print(tp[20000]-tp[19999])
    tF.append(v_back)
    tp.append(0.0)
    while v_back > 0.001 :
        v_back, t = pid.update(0.0, v_back)
        tF.append(v_back)
        tp.append(t-t0)
    plt.plot(tp, tF)
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.grid(True)
    plt.show()
    print(t0)
    print(t-t0)
    # line = interpolate.splrep(tp, tF)
    # print(interpolate.splev(1.0, line))



