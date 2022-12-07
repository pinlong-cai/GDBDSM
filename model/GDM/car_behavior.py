import numpy as np
import random, pdb
from roadInfo import *
from SRP import *

wl = 3.75 
    
def polyfit(status_0, status_t, t): # fit by cubic polynomial function
    e = 0.00001 # avoid 0 as divisor
    px_0,py_0,v0,theta_0 = status_0
    px_t,py_t,vt,theta_t = status_t
    vx_0, vy_0 = v0 * np.cos(theta_0), v0 * np.sin(theta_0)
    vx_t, vy_t = vt * np.cos(theta_t), vt * np.sin(theta_t)
    # P*A = B, A = P^-1 * B
    P = np.matrix([[t**2    , t       , 0       , 0       ],
                   [0       , 0       , t**2    , t       ],
                   [1/3*t**3, 1/2*t**2, 0       , 0       ],
                   [0       , 0       , 1/3*t**3, 1/2*t**2]])
    B = np.matrix([[vx_t-vx_0],[vy_t-vy_0],[px_t-(px_0+vx_0*t)],[py_t-(py_0+vy_0*t)]]) 
    A = np.dot(P.I,B).tolist()
    a1, b1, a2, b2 = A[0][0],A[1][0],A[2][0],A[3][0]
    tra = []
    t_list = np.arange(1, round(t*10)+1)/10
    vx = a1*(t_list)**2 + b1*(t_list) + vx_0
    vy = a2*(t_list)**2 + b2*(t_list) + vy_0
    px = 1/3*a1*(t_list)**3 + 1/2*b1*(t_list)**2 + vx_0*(t_list) + px_0
    py = 1/3*a2*(t_list)**3 + 1/2*b2*(t_list)**2 + vy_0*(t_list) + py_0
    theta = theta_0
    for i in range(len(t_list)):
        if vx[i] > 0.1:
            theta = np.arctan(vy[i]/(vx[i]))  # keep the same with last point when vel is very low
        tra.append([px[i],py[i],np.hypot(vx[i],vy[i]),theta,0,0,0])
    return tra

# determine the terminal point 
def lane_choose(py, car, lane_y):
    # car_status = [car.pos_x, car.pos_y, car.theta]
    # radius_min = car.size[0]/np.sin(PHI_MAX)
    choice_y = []
    lane_y_min = [0,sys.maxsize]
    for it in lane_y:
        if abs(py - it) < lane_y_min[1]:
            lane_y_min = [it,abs(py - it)]
    if lane_y_min[1] < 0.01:
        car.behavior = 0
    for it in lane_y:
        if it < py + wl + 0.001 and it > py - wl - 0.001:
            if car.behavior == 1:
                if it > py:
                    choice_y.append([it, abs(car.pos_y - it)])
            elif car.behavior == -1:
                if it < py:
                    choice_y.append([it, abs(car.pos_y - it)])
            else:
                choice_y.append([it, abs(car.pos_y - it)])   
    choice_y.sort(key=lambda choice_y: choice_y[1])

    return [it[0] for it in choice_y], lane_y_min[0]

def randomsec(sec_list):
    if len(sec_list) == 0:
        return []
    return sec_list[int(random.random()*len(sec_list))]

class CarModule:
    # status: [pos_x, pos_y, vel_x, vel_y, acc_x, acc_y]
    # driving_style: reaction_time, threshold of risk
    def __init__(self, car_id, status, driving_style, color, expect_vel, size = [4,2]):
        self.size = size
        self.id = car_id      
        self.pos_x, self.pos_y, self.vel, self.theta, self.phi, self.acc, self.omega = status 
        self.driving_style = driving_style 
        self.expect_vel = expect_vel
        self.color = color
        self.surCar = {}     # before reaction time  
        self.buffer = 0
        self.behavior = 0
    
    def getStatus(self):
        return [self.pos_x, self.pos_y, self.vel, self.theta, self.phi, self.acc, self.omega][:]
    def setStatus(self,status):
        self.pos_x, self.pos_y, self.vel, self.theta, self.phi, self.acc, self.omega = [round(i,3) for i in status]
    
    # allcars are the status before reaction time
    def getSurCar(self, allcar, car_set, radius = 50):
        self.surCar = {}
        # get other cars by Euclidean distance
        # status: [pos_x, pos_y, vel, theta, phi, acc, omega]
        for car_id in allcar:
            car = car_set[car_id]
            if car.id != self.id:
                if np.hypot(self.pos_x - car.pos_x, self.pos_y - car.pos_y) < radius:
                    self.surCar[car] = traEst(car, self.driving_style[0])

    # planning process
    def planning(self, status, obj_theta = 0):
        px, py, v, theta = status[0], status[1], status[2], status[3]
        v_max = min(v + ACC_MAX * PLAN_TIME, self.expect_vel)
        v_min = max(v - DCC_MAX * PLAN_TIME, 0)
        dv = (v_max - v_min) / 10
        lane_change_buffer = self.buffer 
        last_choice = 0
        # vary_rate = random.random()
        # if vary_rate < 0.2:
        #    v_max = max(min(v * (vary_rate+0.8), VEL_MAX),0)
        try:
            v_range = sorted(np.arange(v_min, v_max+dv, dv), reverse = True)
        except:
            pdb.set_trace()
        for obj_v in v_range:
            phase_dis = (v + obj_v) / 2 * PLAN_TIME
            # if phase_dis < 0.5:
            #    phase_dis, self.vel, v = 0, 0, 0
            if obj_theta == 0:
                obj_lane_y = lane_y
                obj_px = px + phase_dis
            else:
                obj_lane_y = [py + phase_dis * np.sin(obj_theta)]
                obj_px = px + phase_dis * np.cos(obj_theta)
            lane_options, lane_y_close = lane_choose(py, self, obj_lane_y)
            if self.behavior != 0 and abs(lane_y_close - lane_options[0]) < 0.1:
                self.behavior = 0
            if lane_y_close not in lane_options:
                lane_options.append(lane_y_close)
            # if abs(status[2]) < self.expect_vel * 0.2:
            #    lane_options = lane_options[0:1]
            for obj_py in lane_options:
                tra_out = polyfit([px, py, v, theta], [obj_px, obj_py, obj_v, obj_theta], PLAN_TIME)          
                risk = 0
                for other_car in self.surCar: 
                    if isCross(tra_out, self.surCar[other_car]):
                        temp = srpfun(self, tra_out, other_car, self.surCar[other_car])
                        risk = max(temp, risk)  
                if risk == 0:
                    if self.behavior == 0:
                        if abs(obj_py - lane_options[0]) < 0.01:
                            return tra_out
                        else:
                            if abs(self.buffer) >= 5:
                                self.behavior = np.sign(self.buffer)
                                self.buffer = 0
                                return tra_out
                            else:
                                self.buffer = lane_change_buffer + np.sign(obj_py - py)     
                    else:
                        if abs(obj_py - lane_options[0]) < 0.01:
                            return tra_out 
                        elif last_choice == 0:
                            last_choice = tra_out    
                if last_choice == 0 and abs(obj_v - v_min) < dv and abs(obj_py - lane_y_close) < 0.01:
                    last_choice = tra_out
        return last_choice




        
