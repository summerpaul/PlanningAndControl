import numpy as np
import cubic_spline_planner
import matplotlib.pyplot as plt
import math
import scipy.linalg as la
# 参数定义
dt = 0.1 #
L = 2.9 #轴距
Q = np.eye(3)
R = np.eye(2)

max_steer = 1 # rad/s
refSpeed = 0.5 # m/s
refDelta = 0
def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = A.T @ x @ A - A.T @ x @ B @ \
                 la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next

def cal_target_index(pos_x, pos_y, refx, refy):
    dx = [pos_x - icx for icx in refx]
    dy = [pos_y - icy for icy in refy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    return d.index(mind)
def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi
# 计算增益
def calcu_K(A, B, Q, R):
    P = solve_dare(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    return K

def LQR_control(idex, pos_x, pos_y, pos_yaw, v, refx, refy, refyaw):
    x_error = pos_x - refx[idex]
    y_error = pos_y - refy[idex]
    yaw_error = pos_yaw - refyaw[idex]

    # 状态偏差
    X = np.zeros((3,1))
    X[0, 0] = x_error
    X[1, 0] = y_error
    X[2,0] = yaw_error

    # A 矩阵
    A = np.zeros((3, 3))
    A[0, 0] = 1
    A[1, 1] = 1
    A[2, 2] = 1
    A[0, 2] = -v * dt * math.sin(pos_yaw)
    A[1,2] = v * dt * math.cos(pos_yaw)


    # B矩阵
    B = np.zeros((3, 2))
    B[0, 0] = dt * math.cos(pos_yaw)
    B[1, 0] = dt * math.sin(pos_yaw)
    B[2, 0] = dt * math.tan(pos_yaw)/ L
    B[2, 1] = v * dt / (L * math.pow(math.cos(pos_yaw), 2))
    K = calcu_K(A, B, Q, R)

    u = -K @ X
    v_delta = u[0]
    delta = pi_2_pi(u[1])
    return v_delta, delta, yaw_error

def update(idex, pos_x, pos_y, pos_yaw, v, v_delta, delta):
    Delta = refDelta + delta
    x = pos_x + v * math.sin(pos_yaw)*dt
    y = pos_y + v * math.sin(pos_yaw)*dt
    yaw = pos_yaw + v / L * math.tan(Delta) * dt
    v = v + v_delta
    return x, y, yaw, v, Delta

def main():
    ax = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
    goal = [ax[-1], ay[-1]]
    # 对应点的坐标航向角与曲率
    refx, refy, refyaw, refk, refs = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    ref_pose = [refx, refy]

    #赋初值
    pos_x = 0
    pos_y = 0
    pos_yaw = 0
    v = 0.1
    Delta = 0

    pos_actual_x = [pos_x]
    pos_actual_y = [pos_y]
    v_actual = v
    Delta_autual = Delta

    # 循环寻迹
    idex = 1
    while idex < len(refx) -1:
        idex = cal_target_index(pos_x, pos_y, refx, refy)

        v_delta, delta, yaw_error = LQR_control(idex, pos_x, pos_y, pos_yaw, v, refx, refy, refyaw)
        pos_x, pos_y, pos_yaw, v, Delta = update(idex, pos_x, pos_y, pos_yaw, v, v_delta, delta)
        pos_actual_x.append(pos_x)
        pos_actual_y.append(pos_y)
        plt.plot(pos_actual_x, pos_actual_y)
        plt.pause(0.0001)


    plt.plot(refx, refy)



    plt.show()


if __name__ == '__main__':
    main()
# 轨迹处理
