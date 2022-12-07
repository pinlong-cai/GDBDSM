import numpy as np
from roadInfo import * 
from carflow import *
from car_behavior import *
import pdb
import time


lane_width = 3.75
centerline_0 = -lane_width
centerline_1 = 0
centerline_2 = lane_width

for seed in range(100, 101):
    time1 = time.perf_counter()
    decision_cnt = 1
    flow_rate = round(random.random()*0.5+0.5,3)
    lanes = []
    qr, qd, vmin, vmax, running_time, SampleTime = 300, 3600, 10, 20, 1000, 1/10
    CarInfo = []
    # flow from NGSIM data
    qrGroup = [1184, 932, 773.3, 865, 893.3, 882.7, 700]
    for i in range(0,7):
        qr = qrGroup[i] * flow_rate
        lanes.append(generateCarFlow(qr, qd, vmin, vmax, running_time, SampleTime,seed*(i+1)))
        for car in lanes[-1]:
            CarInfo.append([i]+car) # laneid, cnt, arrival_time, velocity, color, style
    CarInfo = sorted(CarInfo, key = lambda x: x[2], reverse = False) # from up to down, lane ids are set as 0，1，2, ...
    lane_num = 7
    car_num = [0 for i in range(lane_num)]
    for car in CarInfo:
        car_num[car[0]] += 1    
    carSpace = {}
    expect_tra = {}
    planning = {}
    car_set = {}
    # the last line is on-ramp
    decision_making = {}
    done_set = []
    for t in range(0,running_time):
        carSpace[t] = {}
        expect_tra[t] = {}
        planning[t] = {}  
        
    for t in range(0,running_time-10):
        for car_info in CarInfo:
            car_id = str((car_info[0], car_info[1])) 
            if car_id not in done_set and -car_info[2] + t > 0 and car_id not in car_set:
                if car_info[0] != 6:
                    # car_id, [pos_x, pos_y, vel, theta, phi, acc, omega], driving_style, color, expect_vel
                    car = CarModule(car_id, init_pos[car_info[0]] + [car_info[3],0,0,0,0], [0.5, 0], car_info[4], car_info[3])
                else:
                    seta = np.arctan(0.03)
                    car = CarModule(car_id, init_pos[car_info[0]] + [car_info[3],0,0,0,0], [0.5, 0], car_info[4], car_info[3]) 
                rt = round(car.driving_style[0]*10) # reaction time
                for i in range(rt):
                    car.pos_x, car.pos_y = car.pos_x+car.vel*np.cos(car.theta)/10, car.pos_y+car.vel*np.sin(car.theta)/10
                    carSpace[t+i][car.id] = car.getStatus()   
                car_set[car_id] = car     

        for car_id in carSpace[t]:
            car = car_set[car_id]
            if car.pos_x > len_lane:
                if car.id not in done_set:
                    done_set.append(car.id)    
            rt = round(car.driving_style[0]*10) # reaction time
            car.setStatus(carSpace[t][car.id])
            car.getSurCar(carSpace[t], car_set)       
            if t+rt < running_time:
                if car.id not in planning[t+rt] and car.id not in done_set:
                    decision_cnt += 1
                    if car.pos_y < 0 and car.pos_x < 220:  # confluence  
                        tra_temp = car.planning(carSpace[t+rt-1][car.id], 0.03)
                    else:
                        tra_temp = car.planning(carSpace[t+rt-1][car.id])
                    for i in range(rt):
                        if t+rt+i < running_time:
                            planning[t+rt+i][car.id] = tra_temp[i][:]

                    for j in range(rt+rt):                    
                        if t+rt+j < running_time:       
                            expect_tra[t+rt+j][car.id] = []
                            for i in range(j,PLAN_TIME*10):
                                if t+rt+i < running_time:                    
                                    if tra_temp[i][0] <= len_lane:
                                        expect_tra[t+rt+j][car.id].append(tra_temp[i][:])
                if car.id in planning[t+rt]:
                    if planning[t+rt][car.id][0] <= len_lane:
                        carSpace[t+rt][car.id] = planning[t+rt][car.id][:]     
                    else:
                        done_set.append(car.id)                    
        if t % 10 == 0:
            print('seed: ', seed, ' flow_rate: ', flow_rate, ' time: ', t)       

    time_cost = time.perf_counter() - time1
    print('time cost: ', time_cost, 'each decision: ' ,  round(time_cost/decision_cnt,3))          

    file = "./result/GDM_"+ str(seed)+ '_' + str(flow_rate) + ".csv"
    output = open(file, "w+")
    for t in range(0,running_time):
        for car_id in carSpace[t]:
            # status: [pos_x, pos_y, vel, theta, phi, acc, omega]
            status = carSpace[t][car_id]
            output.write('%d,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n'%(t,car_id,status[0],status[1],status[2],status[3],status[4],status[5]))
    output.close()

    print('------------------------------------------\n')




