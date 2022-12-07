import numpy as np
import os
from roadInfo import * 
from carflow import *
from get_flow import *
from car_behavior import *
import time
import pickle
# info process
# map, straight roads with three lanes


time1 = time.perf_counter()

lanes = []
# Frames per second is 1/SampleTime, and the unit of running_time is 0.1s. 
# running_time for 15 minutes more than 15 * 60 / SampleTime.
qr, qd, vmin, vmax, running_time, SampleTime = 300, 3600, 10, 20, 100, 1/10
CarInfo = []


# flow calculated from NGSIM data (option 1)
# lane from left to right (from up to down) , lane ids are set as 0，1，2, ...

# generate vehicle randomly
# qrGroup = [1184, 932, 773.3, 865, 893.3, 882.7, 700]
# qr_cnt = [0,0,0,0,0,0,0] # vehicel count
# for i in range(0,7):
#    qr = qrGroup[i]
#    lanes.append(generateCarFlow(qr, qd, vmin, vmax, running_time, SampleTime, i))    
#    for car in lanes[-1]:
#        CarInfo.append([i]+car) # laneid, cnt, arrival_time, velocity, color, style
        # qr_cnt[i] += 1
# CarInfo = sorted(CarInfo, key = lambda x: x[2], reverse = False) 
# print(qr_cnt, [i/(running_time/10)*3600 for i in qr_cnt])

# generate vehicle from real data (NGSIM) (option 2)
period  = 3 # period is in [1, 2, 3]
inflie = '../../I80_data_process/carflow_from_data/init_vehicle_' + str(period) + '.csv'
CarInfo = getCarFlow(inflie)
random.seed(10)


lane_num = 7    # the last line is on-ramp
carSpace = {}   # record vehicle states
expect_tra = {} # record expected trajectory for drawing guidance lines
planning = {}   # record planning results 
car_set = {}    # record car id
done_set = []   # record the vehicle completing the travels

for t in range(0,running_time):
    carSpace[t] = {}
    expect_tra[t] = {}
    planning[t] = {}

for t in range(0,running_time-10): 
    # initilize the vehicles when they arrive the origins
    for car_info in CarInfo:
        car_id = str((car_info[0], car_info[1])) 
        if car_id not in done_set and -car_info[2] + t > 0 and car_id not in car_set:
            if car_info[0] != 6:
                # car_id, [pos_x, pos_y, vel, theta, phi, acc, omega], driving_style, color, expect_vel
                car = CarModule(car_id, init_pos[car_info[0]] + [car_info[3],0,0,0,0], [0.5, 0], car_info[4], random.random()*10+10)
            else: # onramp
                seta = np.arctan(0.03)
                car = CarModule(car_id, init_pos[car_info[0]] + [car_info[3],0,0,0,0], [0.5, 0], car_info[4], random.random()*10+10) 
            rt = round(car.driving_style[0]*10) # reaction time
            # keep moving along stright lines
            for i in range(rt):
                car.pos_x, car.pos_y = car.pos_x+car.vel*np.cos(car.theta)/10, car.pos_y+car.vel*np.sin(car.theta)/10
                carSpace[t+i][car.id] = car.getStatus()   
            car_set[car_id] = car     

    # planning for each vehicle 
    for car_id in carSpace[t]:
        car = car_set[car_id]
        if car.pos_x > len_lane:
            if car.id not in done_set:
                done_set.append(car.id)    
        rt = round(car.driving_style[0]*10) # reaction time
        # perception
        car.setStatus(carSpace[t][car.id]) # ego status
        car.getSurCar(carSpace[t],car_set) # other statuses      
        if t+rt < running_time:
            # planning process
            if car.id not in planning[t+rt] and car.id not in done_set:
                if car.pos_y < 0 :  # confluence of on ramp
                    tra_temp = car.planning(carSpace[t+rt-1][car.id], 0.03)
                else:
                    tra_temp = car.planning(carSpace[t+rt-1][car.id])
                for i in range(rt):
                    if t+rt+i < running_time: 
                        planning[t+rt+i][car.id] = tra_temp[i][:]
                for j in range(rt):                    
                    if t+rt+j < running_time:       
                        expect_tra[t+rt+j][car.id] = []
                        for i in range(j,PLAN_TIME*10):
                            if t+rt+i < running_time:                    
                                if tra_temp[i][0] <= len_lane:
                                    expect_tra[t+rt+j][car.id].append(tra_temp[i][:])
            # upate the state in car_status
            if car.id in planning[t+rt]:
                if planning[t+rt][car.id][0] <= len_lane:
                    car_status = planning[t+rt][car.id][0:4]
                    for other_car in car.surCar: # avoid conflict to ensure safety
                        if other_car.id in carSpace[t+rt]:
                            other_status = carSpace[t+rt][other_car.id][0:4]
                            if car_status[0] < other_status[0] and isCross_point(car_status, other_status): # radius
                                planning[t+rt][car.id] = carSpace[t+rt-1][car.id][:]
                    carSpace[t+rt][car.id] = planning[t+rt][car.id][:]     
                else:
                    done_set.append(car.id)         
    if t % 10 == 0:
        print('GDM: period = %d, time = %d'%(period, t))

print('computing time:', time.perf_counter() - time1, 's')

# save velocity

vel_url = 'velocity_sum/'
if not os.path.exists(vel_url):  
    os.makedirs(vel_url)

outfile = vel_url + "simuilation_velocity_GDM_" + str(period) + ".csv"
output = open(outfile, "w+")
for t in range(0,running_time):
    for car_id in carSpace[t]:
        v = np.hypot(carSpace[t][car_id][2], carSpace[t][car_id][3])
        output.write('%.2f\n'%(v))


# save traffic states during the simulation
pkl_url = 'pkl_period_' + str(period) + '/'

if not os.path.exists(pkl_url):  
    os.makedirs(pkl_url)


with open(pkl_url + 'carset.pickle', 'wb') as file:
    pickle.dump(car_set, file)
with open(pkl_url + 'carset.pickle', 'rb') as file:
    car_set = pickle.load(file)

with open(pkl_url + 'carSpace.pickle', 'wb') as file:
    pickle.dump(carSpace, file)
with open(pkl_url + 'carSpace.pickle', 'rb') as file:
    carSpace = pickle.load(file)

with open(pkl_url + 'expect_tra.pickle', 'wb') as file:
    pickle.dump(expect_tra, file)
with open(pkl_url + 'expect_tra.pickle', 'rb') as file:
    expect_tra = pickle.load(file)

# pygame
def drawText(screen,text,posx,posy,textHeight=32,fontColor=(0,0,0),backgroudColor=(255,255,255)):
    myfont=pygame.font.Font('freesansbold.ttf',textHeight)
    textSurfaceObj=myfont.render(text, True, fontColor, backgroudColor)
    textRectObj=textSurfaceObj.get_rect()
    textRectObj.center=(posx,posy)
    screen.blit(textSurfaceObj,textRectObj.center)
if 1:
    import pygame, time
    pygame.init()
    w = 1100*2
    h = 400*2
    def cps(x, y): # change point size
        x = x*4 + 40
        y = -(y*4) + h/2
        return x,y

    screen = pygame.display.set_mode((w, h))
    cnt = 0
    for t in range(0,running_time):
        time.sleep(0.1)            
        screen.fill((85,87,83)) 
        color = [220,220,220]

        # draw road lines
        width = 1
        solidline = [
                    ((100,-6),(325,0)),
                    ((100,-3),(200,0)),
                    ((0,0),(200,0)),
                    ((325,0),(len_lane,0)),
                    ((0,6*wl),(len_lane,6*wl))
                    ]
        for line in solidline:        
            point0_x, point0_y = line[0][0], line[0][1]
            point1_x, point1_y = line[1][0], line[1][1]
            pygame.draw.line(screen,color,cps(point0_x, point0_y),cps(point1_x, point1_y),width)   
        dotline = [
                    ((0,1*wl),(len_lane,1*wl)),
                    ((0,2*wl),(len_lane,2*wl)),
                    ((0,3*wl),(len_lane,3*wl)),
                    ((0,4*wl),(len_lane,4*wl)),
                    ((0,5*wl),(len_lane,5*wl)),
                    ]
        width = 1
        num_sec = 101
        for line in dotline:        
            for dot in range(0,num_sec):
                if dot % 2 == 0:
                    point0_x, point0_y = (1-dot/num_sec)*line[0][0] + (dot/num_sec)*line[1][0], (1-dot/num_sec)*line[0][1] + (dot/num_sec)*line[1][1]
                    point1_x, point1_y = (1-(dot+1)/num_sec)*line[0][0] + ((dot+1)/num_sec)*line[1][0], (1-(dot+1)/num_sec)*line[0][1] + ((dot+1)/num_sec)*line[1][1]
                    pygame.draw.line(screen,color,cps(point0_x, point0_y),cps(point1_x, point1_y),width)           

        # draw vehicles and guidance lines
        for car_id, status in carSpace[t].items():
            car = car_set[car_id]
            colorGroup = [[230,100,100],[100,230,100],[100,100,230]]
            it = CooGet([status[0], status[1], status[3]])
            points = [(cps(it[0][0], it[0][1])),
                      (cps(it[1][0], it[1][1])),
                      (cps(it[2][0], it[2][1])),
                      (cps(it[3][0], it[3][1]))]
            pygame.draw.polygon(screen, car.color, points, 0)
            if car_id in expect_tra[t]:
                for point in expect_tra[t][car_id]:
                    pygame.draw.circle(screen,car.color,cps(point[0],point[1]),1,1)
            
            # [posx, posy] = cps(status[0],status[1]) 
            # drawText(screen,str(car[0])+','+str(car[1]),posx-8,posy-5,textHeight=16,fontColor=(0,0,0),backgroudColor=None) # draw vehicle id
        pygame.display.update()    
        pygame.display.flip()    
        # 8 - loop through the events
        for event in pygame.event.get():
            # check if the event is the X button 
            if event.type==pygame.QUIT:
                # if it is quit the game
                pygame.quit() 
                exit(0)
