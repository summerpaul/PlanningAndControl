import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import math
sys.path.append("../curves/")
sys.path.append("../draw/")
try:
    import cubic_spline_planner
    import draw.drawCar as draw
except ImportError:
    raise
"""
A=[
    [1, 0, -T * v *sin(yaw)],
    [0, 1, T * V * cos(yaw)],
    [0, 0, 1]
]

B = [
     [T * cos(yaw), 0],
     [T * sin(yaw), 0],
     [T * tan(yaw) / L, T * v /(L * (cos(yaw)^2)]
    ]
    X(k+1) = AX(K)+Bu(k)
"""



def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi
def calc_target_index(pos_x,pos_y, refPos_x,refPos_y):
    dist=[]
    for i in range(0, len(refPos_x)):
        dx = pos_x - refPos_x[i]
        dy = pos_y - refPos_y[i]
        diff = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        dist.append(diff)
    mind = min(dist)
    return dist.index(mind)

def calcu_K(A,B,Q,R):
    iter_max = 5000
    epsilon = 0.001
    P_old = Q
    AT = A.T
    BT = B.T
    for i in range(iter_max):
        P_new = AT @ P_old @ A - AT @ P_old @ B @ np.linalg.inv(R + BT @ P_old @ B) @ (BT @ P_old @ A) + Q
        if (abs(P_new - P_old)).max() < epsilon:
            break
        else:
            P_old = P_new
        P = P_new
    K = np.linalg.inv(BT @ P @ B + R) @ (BT @ P @ A)
    return K

def LQR_contorl(idx,x,y,v,yaw,refPos_x,refPos_y,refPos_yaw,refPos_Delta,refK,L,Q,R,dt):
    x_r = refPos_x[idx]
    y_r = refPos_y[idx]
    heading_r = refPos_yaw[idx]
    delta_r = math.atan2(refK[idx]*L, 1)
    x_error  = x - x_r
    y_error = y - y_r
    yaw_error =  yaw - heading_r
    latError = y_error*math.cos(heading_r) - x_error*math.sin(heading_r)

    X = np.zeros((3, 1))
    X[0, 0] = x_error
    X[1, 0] = y_error
    X[2, 0] = yaw_error

    #由状态方程矩阵系数，计算K
    """
A=[
    [1, 0, -T * v *sin(yaw)],
    [0, 1, T * V * cos(yaw)],
    [0, 0, 1]
]

B = [
     [T * cos(yaw), 0],
     [T * sin(yaw), 0],
     [T * tan(yaw) / L, T * v /(L * (cos(yaw)^2)]
    ]
    X(k+1) = AX(K)+Bu(k)
"""
    A = np.zeros((3, 3))
    A[0, 0] = 1
    A[0, 2] = -v* dt * math.sin(heading_r)
    A[1, 1] = 1
    A[1, 2] = v * dt * math.cos(heading_r)
    A[2, 2] = 1

    B = np.zeros((3, 2))
    B[0, 0] = dt * math.cos(heading_r)
    B[1, 0] = dt * math.sin(heading_r)
    B[2, 0] =  dt * math.tan(heading_r)/L
    B[2, 1] = v*dt/(L * (math.pow(math.cos(delta_r),2)))
    """
    A = [1,  0,  -v*dt*sin(heading_r);
        0,  1,  v * dt * cos(heading_r);
            0,  0,  1];
    B = [dt * cos(heading_r),    0;
     dt * sin(heading_r),    0;
     dt * tan(heading_r)/L,  v*dt/(L * cos(delta_r)^2)];
    """



    K = calcu_K(A,B,Q,R)

    u = -K @ X
    v_delta = u[0]
    Delta_delta = pi_2_pi(u[1])
    return v_delta,Delta_delta,delta_r,latError

def update(x, y, yaw, v, v_delta,Delta_delta,dt,L,refSpeed,refDelta):
    Delta = refDelta + Delta_delta
    x = x + v * math.cos(yaw) * dt
    y = y + v * math.sin(yaw) * dt
    yaw = yaw + v / L * math.tan(Delta) * dt
    v = v + v_delta
    return x, y, yaw, v, Delta

def main():
    ax = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
    goal = [ax[-1], ay[-1]]
    refPos_x, refPos_y, refHeading, refK, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    dt = 0.1
    L = 2.9
    Q =  np.eye(3)
    R = np.eye(2)
    refPos_Delta = []
    for i in range(len(refK)):
        refPos_Delta.append(L*refK[i])

    refSpeed = 40 / 3.6

    x = ax[0] + 0.5
    y = ay[1] + 0.5
    yaw = refHeading[1] + 0.02
    v = 0
    Delta = 0
    idx = 0
    pos_actual_x = [x]
    pos_actual_y = [y]
    v_actual = v
    Delta_actual = Delta
    idx_actual = 1
    latError_LQR = []
    # Q = np.ones((3, 3))
    # R = np.ones((1, 1))

    while idx < len(refPos_x) - 1:
        idx = calc_target_index(x, y, refPos_x, refPos_y)

        v_delta, delta, delta_r, latError = LQR_contorl(idx,x,y,v,yaw,refPos_x,refPos_y,refHeading,refPos_Delta,refK,L,Q,R,dt)

        if abs(latError > 3):
            print("误差过大,跳出程序")
            break
        x,y,yaw,v,Delta = update(x,y,yaw,v, v_delta,delta, dt,L, refSpeed,delta_r)
        plt.plot(refPos_x, refPos_y, "-r")
        pos_actual_x.append(x)
        pos_actual_y.append(y)
        plt.plot(pos_actual_x, pos_actual_y,"ob",)
        plt.pause(0.0001)
        # draw.draw_car(x, y, yaw, Delta)


    # plt.plot(pos_actual_x, pos_actual_y)
    plt.show()
if __name__ == '__main__':
    main()
