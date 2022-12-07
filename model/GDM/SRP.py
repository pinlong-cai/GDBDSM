# SRP: subjective risk perception
# SRP is applied as the cost function in driving decision
# decision-making process with SRP is a imitation driving model

import numpy as np
import sys

from dubins import dubins


SAFE_DIS_x = 8  # meter
SAFE_DIS_y = 3  # meter
PHI_MAX = 0.7 # rad
OME_MAX = 0.3 # rad/s
ACC_MAX = 3   # m/s^2
DCC_MAX = 8   # m/s^2
VEL_MAX = 20  # m/s
PLAN_TIME = 2
R_MAX = 0.6
R_MIN = 0.3

def sign(x):
    if x >= 0:
        return 1
    else:
        return 0

def func2r(x):
    x = max(min(x,10),0)
    return 1-np.exp(-x)

def isCross_point(p0,p1):
    relative_angle = np.arctan((p1[1] - p0[1]) / (abs(p1[0]-p0[0])+0.001))
    relative_dis = np.hypot(p1[0]-p0[0], p1[1]-p0[1])
    if p0[0] < p1[1]:
        cos_v = abs(np.cos(relative_angle-p0[3]))
        sin_v = abs(np.sin(relative_angle-p0[3]))
    else:
        cos_v = abs(np.cos(relative_angle-p1[3]))
        sin_v = abs(np.sin(relative_angle-p1[3]))            
    if relative_dis * sin_v < SAFE_DIS_y and relative_dis * cos_v < SAFE_DIS_x:
        return True

def traEst(car, rt, period = PLAN_TIME * 10):
    tra = []
    px, py, vel, theta = car.pos_x, car.pos_y, car.vel, car.theta
    px_list = px + vel * np.cos(theta) / 10 * np.arange(0, period+round(rt*10))
    py_list = py + vel * np.sin(theta) / 10 * np.arange(0, period+round(rt*10))
    for t in range(period+round(rt*10)):
        tra.append([px_list[t],py_list[t],vel,theta])
    return tra

def isCross(ego_expect, other_estimate):
    for p0 in ego_expect:
        for p1 in other_estimate:
            if isCross_point(p0,p1):
                return True
    return False

def extremeAction(expect_point, other_car):   
    x = np.arange(other_car.pos_x - SAFE_DIS_x, other_car.pos_x + SAFE_DIS_x, 1)
    y = np.arange(other_car.pos_y - SAFE_DIS_y, other_car.pos_y + SAFE_DIS_y, 1)
    x_len, y_len = len(x), len(y)
    x = np.dot(x.reshape(x_len,1), np.ones(y_len).reshape(1,y_len))
    y = np.dot(np.ones(x_len).reshape(x_len,1),y.reshape(1,y_len))
    dis = np.hypot(x - expect_point[0], y - expect_point[1])
    index = np.where(dis == dis.min())
    key_x, key_y = x[index[0][0]][index[1][0]], y[index[0][0]][index[1][0]] 
    other_car_status = [key_x, key_y, other_car.theta]
    radius_min = other_car.size[0]/np.sin(PHI_MAX)
    tra_cur = dubins(other_car_status, expect_point, radius_min)
    return tra_cur
                
def srpfun(ego_car, expect_tra, other_car, other_estimate):
    rt = ego_car.driving_style[0] # reaction time 
    dis_acc = ego_car.vel * rt
    for t in range(0, len(expect_tra)):
        risk_1, risk_2 = 0, 0
        ego_est_x, ego_est_y, ego_est_v, ego_est_theta = expect_tra[t][0:4]
        other_est_x, other_est_y, other_est_v, other_est_theta = other_estimate[t+round(rt*10)][0:4]
        if t > 0:
            dis_acc += np.hypot(ego_est_x-expect_tra[t-1][0], ego_est_y-expect_tra[t-1][1])
            
        relative_dis = np.hypot(ego_est_x - other_est_x, ego_est_y - other_est_y)

        # backward risk when change lane
        if max([it[3] for it in expect_tra]) - min([it[3] for it in expect_tra]) > 0.1 and min([abs(it[3]) for it in other_estimate]) < 0.01:
            if relative_dis < SAFE_DIS_x:
                return 1

        # forward risk in currend lane
        if other_car.pos_x > ego_car.pos_x and abs(other_car.pos_y - ego_car.pos_y) < SAFE_DIS_y:
            relative_angle = np.arctan((ego_est_y - other_est_y) / (ego_est_x-other_est_x+0.001))
            other_vel_map = other_est_v * np.cos(abs(other_est_theta - relative_angle))            
            risk_1 = func2r((2 * rt * ego_est_v + ego_est_v ** 2 / (2 * DCC_MAX) - sign(other_vel_map)*(other_vel_map**2/(2 * DCC_MAX)))/ max(relative_dis, 0.001)) # * ego_est_v / VEL_MAX 
            if isCross_point(expect_tra[t][0:4],other_estimate[t+round(rt*10)][0:4]):
                return 1

        # forward risk in objective lane
        if other_car.pos_x < ego_est_x + SAFE_DIS_x and abs(ego_est_theta) > 0.01 and abs(ego_est_theta - other_est_theta) > 0.01:
            d_theta = np.pi/4
            tra_cur = sys.maxsize
            for other_theta in np.arange(-np.pi/2, np.pi/2+d_theta, d_theta):
                temp = extremeAction([ego_est_x, ego_est_y, other_theta], other_car) 
                if temp < tra_cur:
                    tra_cur = temp
                    other_vel_map = other_est_v * np.cos(abs(ego_est_theta - other_theta))
            if tra_cur/(other_est_v+0.0001) <= t/10 + 2 * rt : # 1s is safe time gap
                risk_2 = func2r((2 * rt * ego_est_v + ego_est_v ** 2 / (2 * DCC_MAX) - sign(other_vel_map)*other_vel_map**2/(2 * DCC_MAX))/ (dis_acc + 0.0001))# * ego_est_v / VEL_MAX
            else:
                risk_2 = 0        

        if max(risk_1, risk_2) > R_MAX - (R_MAX - R_MIN) * t/len(expect_tra):
            return max(risk_1, risk_2)   
    return 0


