import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import math
sys.path.append("../curves/")
try:
    import cubic_spline_planner
except ImportError:
    raise
def calc_target_index(pos_x,pos_y, refPos_x,refPos_y):
    dist=[]
    for i in range(0, len(refPos_x)):
        dx = pos_x - refPos_x[i]
        dy = pos_y - refPos_y[i]
        diff = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        dist.append(diff)
    mind = min(dist)
    return dist.index(mind)


def main():
    ax = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
    goal = [ax[-1], ay[-1]]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    dt = 0.1;
    L = 2.9;
    Q = 100 * np.eye(3);
    R = np.eye(2) * 2;
    refPos_Delta = math.atan(L * s[0]);

    refSpeed = 40 / 3.6;

    x = ax[0] + 0.5;
    y = ay[1] + 0.5;
    yaw = cyaw[1] + 0.02;
    v = 10;
    Delta = 0;
    idx = 0;
    pos_actual = [x, y];
    v_actual = v;
    Delta_actual = Delta;
    idx_actual = 1;
    latError_LQR = [];

    while idx < len(cx) - 1:
        idx =


    plt.plot(cx, cy)
    plt.show()
if __name__ == '__main__':
    main()