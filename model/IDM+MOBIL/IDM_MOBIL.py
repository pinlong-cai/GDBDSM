import numpy as np
import matplotlib.pyplot as plt 
from roadInfo import * 
from carflow import *
import pygame, time
import random
# info process

# map, straight roads with three lanes
lane_width = 3.5
centerline_0 = -lane_width
centerline_1 = 0
centerline_2 = lane_width
safe_gap = 10
safe_gap2 = 30
# percepive

distance_range = 30
ov = 20
change_rate = 0.5

def getCarFlow(file_url):   
    infile = open(file_url) 
    text_line = infile.readline()
    colorGroup = [[230,100,100],[100,230,100],[100,100,230],
                      [230,200,100],[100,230,200],[200,100,230]]
    
    CarInfo = []
    cnt = [0] * 7
    while(len(text_line) > 1): 
        sub = text_line.strip('\n').split(',')
        lane_id, arrival_time, vel = int(sub[0]), int(sub[1]), float(sub[2])
        c = colorGroup[random.randint(0,5)]
        s = random.randint(0,4)
        CarInfo.append([lane_id, cnt[lane_id], arrival_time, max(vel,0.01), c, s])
        cnt[lane_id] += 1 
        text_line = infile.readline()
    return CarInfo

def get_traffic_status(ego_id, allcars, per_radius = 50):    
    pos = allcars[ego_id]
    ego_x, ego_y = pos[0], pos[1]
    sur_status = []
    for car in allcars:
        other_x, other_y = allcars[car][0], allcars[car][1]
        if car != ego_id and distance([ego_x, ego_y], [other_x, other_y]) < per_radius:
            sur_status.append(car)
    return sur_status


def IDM(v_cur, v_lead, v_des, s_cur):
    '''
    :reference: https://traffic-simulation.de/info/info_IDM.html
    :return: acceleration
    '''
    a_max = 10
    T = 1.0
    s_0 = 2.0
    acc = 3.0
    dec = 6.0

    d_v = v_cur - v_lead
    if s_cur == 0:
        s_cur = 0.00001
    s_star = s_0 + max(0, (v_cur * T + (v_cur * d_v) / (2 * math.sqrt(acc * dec))))
    a = a_max * (1 - pow(v_cur / v_des, 4) - pow(s_star / s_cur, 2))
    if a > 0:
        a = min(acc, a)
    if a < 0:
        a = max(-dec, a)
    return a

def mobil(ego_status, curlane_pre_statues, tolane_pre_status, tolane_fol_status):
    #status include velocity, posistion
    vxy, fv_vxy, relative_dis = ego_status[0], curlane_pre_statues[0], curlane_pre_statues[1] - ego_status[1]
    a_ego_cf = IDM(vxy, fv_vxy, expect_vel[car_id], relative_dis)
    vxy, fv_vxy, relative_dis = ego_status[0], tolane_pre_status[0], tolane_pre_status[1] - ego_status[1]
    a_ego_lc = IDM(vxy, fv_vxy, expect_vel[car_id], relative_dis)
    vxy, fv_vxy, relative_dis = tolane_fol_status[0], tolane_pre_status[0], tolane_pre_status[1] - tolane_fol_status[1]
    a_fol_cf = IDM(vxy, fv_vxy, expect_vel[car_id], relative_dis)
    vxy, fv_vxy, relative_dis = tolane_fol_status[0], ego_status[0], ego_status[1] - tolane_fol_status[1]
    a_fol_lc = IDM(vxy, fv_vxy, expect_vel[car_id], relative_dis)
    b_safe = -10
    c_polite = 0.5
    a_thr = 0.5
    safe_score = a_fol_lc - b_safe
    incent_score = (a_ego_lc - a_ego_cf) - c_polite * (a_fol_cf - a_fol_lc) - a_thr
    return safe_score > 0 and incent_score > 0

lanes = []
qr, qd, vmin, vmax, runingtime, SampleTime = 300, 3600, 10, 20, 10000, 1/10
CarInfo = []

period = 3
inflie = '../../I80_data_process/carflow_from_data/init_vehicle_' + str(period) + '.csv'

CarInfo = getCarFlow(inflie)
random.seed(0)

colorset = {}
expect_vel = {}
for car in CarInfo:
    lane_id, cnt, color = car[0], car[1], car[4]
    colorset[(lane_id,cnt)] = color
    expect_vel[(lane_id,cnt)] = random.random()*10+10


lane_num = 7
car_num = [0 for i in range(lane_num)]
for car in CarInfo:
    car_num[car[0]] += 1
carSpace = {}
# the last line is on-ramp
init_pos = [[0,wl*5.5],[0,wl*4.5],[0,wl*3.5],
            [0,wl*2.5],[0,wl*1.5],[0,wl*0.5],
            [100,-3-wl*0.5],
            ]
decision_making = {}
lane_change = {}
done_set =[]
for t in range(0,runingtime):
    carSpace[t] = {}
for t in range(0,runingtime):
    for car in CarInfo:
        car_id = (car[0], car[1]) 
        if car_id not in done_set and -car[2] + t > 0 and car_id not in carSpace[t-1]:
            if car_id[0] != 6:
                carSpace[t][car_id] = init_pos[car[0]] + [car[3],0] +  [0,0] # pos, vel, acc
            else:
                seta = np.arctan(0.03)
                carSpace[t][car_id] = init_pos[car[0]] + [car[3]*np.cos(seta),car[3]*np.sin(seta)] +  [0,0] # pos, vel, acc
    if t > 0:
        for car_id, v in carSpace[t-1].items():
            if car_id in carSpace[t-1] and carSpace[t-1][car_id][0] > len_lane:
                if car_id not in done_set:
                    done_set.append(car_id)   
                for i in range(0,10):
                    if t+i < runingtime and car_id in carSpace[t+i]:
                        del carSpace[t+i][car_id]
            if car_id in decision_making and t in decision_making[car_id]:
                continue
            elif car_id not in done_set:
                sur_status = get_traffic_status(car_id, carSpace[t-1])
                px, py, vx, vy, ax, ay = v[0], v[1], v[2], v[3], v[4], v[5]
                if car_id[0] == 6:  # vehicle in ramp
                    if px < 150:
                        fv_px = 10000
                        FV = 0
                        for car in sur_status:
                            if car[0] == car_id[0] and carSpace[t-1][car][0] > px: # Front vehicles in the same lane
                                if carSpace[t-1][car][0] < fv_px:
                                    FV = car
                                    fv_px = carSpace[t-1][car][0]
                        if FV:
                            [fv_px, fv_py, fv_vx, fv_vy, fv_ax, fv_ay] = carSpace[t-1][FV]
                            relative_dis = np.hypot(fv_px, fv_py) - np.hypot(px, py)
                            fv_vxy = np.hypot(fv_vx, fv_vy) 
                        else:
                            fv_vxy = expect_vel[car_id]  
                            relative_dis = 100  
                        vxy = np.hypot(vx, vy)           
                        axy = IDM(vxy, fv_vxy, expect_vel[car_id], relative_dis)
                        seta = np.arctan(vy/(vx+0.0001))
                        vxy = min(max(vxy + axy/10, 0), expect_vel[car_id])
                        vx, vy = vxy * np.cos(seta), vxy * np.sin(seta)
                        ax, ay = axy * np.cos(seta), axy * np.sin(seta)
                        px = px + (vx + vx + ax/10)*1/2*1/10
                        py = py + (vy + vy + ay/10)*1/2*1/10
                        carSpace[t][car_id] = [px, py, vx, vy, ax, ay]
                    else:
                        fv_px = 10000
                        FV = 0
                        for car in sur_status:
                            if (car[0] == car_id[0] or car[0] == car_id[0]-1) and carSpace[t-1][car][0] > px: # Front vehicles in the same lane
                                if carSpace[t-1][car][0] < fv_px:
                                    FV = car
                                    fv_px = carSpace[t-1][car][0]
                        if FV:
                            [fv_px, fv_py, fv_vx, fv_vy, fv_ax, fv_ay] = carSpace[t-1][FV]
                            relative_dis = np.hypot(fv_px, fv_py) - np.hypot(px, py)
                            fv_vxy = np.hypot(fv_vx, fv_vy) 
                        else:
                            fv_vxy = expect_vel[car_id]  
                            relative_dis = 100  
                        vxy = np.hypot(vx, vy)                    
                        axy = IDM(vxy, fv_vxy, expect_vel[car_id], relative_dis)
                        seta = np.arctan(vy/(vx+0.0001))
                        vxy = min(max(vxy + axy/10, 0), expect_vel[car_id])
                        vx, vy = vxy * np.cos(seta), vxy * np.sin(seta)
                        ax, ay = axy * np.cos(seta), axy * np.sin(seta)
                        px = px + (vx + vx + ax/10)*1/2*1/10
                        py = py + (vy + vy + ay/10)*1/2*1/10
                        carSpace[t][car_id] = [px, py, vx, vy, ax, ay]
                        if 1:
                            py = carSpace[t][car_id][1]
                            if py >= init_pos[5][1]:  # Vehicles on merging lanes
                                del carSpace[t][car_id]
                                car_num[car_id[0]-1] += 1
                                car_id2 = (car_id[0]-1, car_num[car_id[0]-1])
                                colorset[car_id2] = colorset[car_id]
                                expect_vel[car_id2] = expect_vel[car_id]              
                                seta = np.arctan(0.03)
                                px, py, vx, vy, ax, ay = v[0], init_pos[5][1], v[2]/np.cos(seta), 0, v[4], v[5]
                                carSpace[t][car_id2] = [px, py, vx, vy, ax, ay]
                                done_set.append(car_id)
                                car_fv_vxyid = (car_id2[0], car_id2[1])  
                else:                    
                    fv_px = 10000
                    FV = -1
                    right_fv_px = 10000
                    right_bv_px = -10000
                    left_fv_px = 10000
                    left_bv_px = -10000
                    right_lane = [0,0]
                    left_lane = [0,0]
                    for car in sur_status:
                        if (car[0] == car_id[0] or (px > 200 and car_id[0] == 5 and car[0] == car_id[0]+1) ) and carSpace[t-1][car][0] > px: # 同一车道的前面车辆
                            if carSpace[t-1][car][0] < fv_px:
                                FV = car
                                fv_px = carSpace[t-1][car][0]
                        if car_id[0] < 5: # Turn right
                            if (car[0] == car_id[0]+1 or (px > 200 and car_id[0] == 4 and car[0] == car_id[0]+2))  and carSpace[t-1][car][0] > px: # 右转前车
                                if carSpace[t-1][car][0] < right_fv_px:
                                    right_lane[0] = car
                                    right_fv_px = carSpace[t-1][car][0]
                            elif (car[0] == car_id[0]+1 or (px > 200 and car_id[0] == 4 and car[0] == car_id[0]+2) )  and carSpace[t-1][car][0] <= px: # 右转后车
                                if carSpace[t-1][car][0] > right_bv_px:
                                    right_lane[1] = car
                                    right_bv_px = carSpace[t-1][car][0]
                            pass                        
                        if car_id[0] > 0: # Turn left
                            if car[0] == car_id[0]-1 and carSpace[t-1][car][0] > px: # ahead in the left-turning lane
                                if carSpace[t-1][car][0] < left_fv_px:
                                    left_lane[0] = car
                                    left_fv_px = carSpace[t-1][car][0]
                            elif car[0] == car_id[0]-1 and carSpace[t-1][car][0] <= px: # behind in the left-turning lane
                                if carSpace[t-1][car][0] > left_bv_px:
                                    left_lane[1] = car
                                    left_bv_px = carSpace[t-1][car][0]
                    if fv_px == 10000:
                        ax = IDM(vx, 20, expect_vel[car_id], 10000)
                    else:
                        [fv_px, fv_py, fv_vx, fv_vy, fv_ax, fv_ay] = carSpace[t-1][FV]
                        ax = IDM(vx, fv_vx, expect_vel[car_id], fv_px - px)
                    if ax < -0.001 and vx > 5 and t%10 == 0:
                        if left_lane[0]:
                            left_lane_pre = carSpace[t-1][left_lane[0]]
                            left_lane_pre_status = [left_lane_pre[2], left_lane_pre[0]]  
                        else:
                            left_lane_pre_status = [20,10000]
                        if left_lane[1]:
                            left_lane_fol = carSpace[t-1][left_lane[1]]   
                            left_lane_fol_status = [left_lane_fol[2], left_lane_fol[0]]  
                        else:
                            left_lane_fol_status = [0,-10000]         
                        left_turning_decision = mobil([vx,px], [fv_vx,fv_px], left_lane_pre_status, left_lane_fol_status)
                        if right_lane[0]:
                            right_lane_pre = carSpace[t-1][right_lane[0]]
                            right_lane_pre_status = [right_lane_pre[2], right_lane_pre[0]]  
                        else:
                            right_lane_pre_status = [20,10000]
                        if right_lane[1]:
                            right_lane_fol = carSpace[t-1][right_lane[1]]   
                            right_lane_fol_status = [right_lane_fol[2], right_lane_fol[0]]  
                        else:
                            right_lane_fol_status = [0,-10000]                     
                        right_turning_decision = mobil([vx,px], [fv_vx,fv_px], right_lane_pre_status, right_lane_fol_status)
                        if car_id[0] > 0  and (FV not in lane_change or t not in lane_change[FV]) and left_turning_decision:
                            line = left_turning([px, py], vx)
                            car_num[car_id[0]-1] += 1
                            car_id2 = (car_id[0]-1, car_num[car_id[0]-1])
                            colorset[car_id2] = colorset[car_id]
                            expect_vel[car_id2] = expect_vel[car_id]
                            for i in range(0,turning_time*10+1):
                                seta = math.atan(line[i][2])
                                if t+i < runingtime:
                                    if i < turning_time*10/2:
                                        carSpace[t+i][car_id] = [line[i][0], line[i][1], vx*np.cos(seta), vx*np.sin(seta), 0, 0]
                                    elif i < turning_time*10:
                                        carSpace[t+i][car_id2] = [line[i][0], line[i][1], vx*np.cos(seta), vx*np.sin(seta), 0, 0]
                                    else:
                                        carSpace[t+i][car_id2] = [line[i][0], line[i][1], vx, 0, 0, 0]
                                    if car_id not in decision_making:
                                        decision_making[car_id] = []
                                    decision_making[car_id].append(t+i)
                                    if car_id2 not in decision_making:
                                        decision_making[car_id2] = []
                                    decision_making[car_id2].append(t+i)

                                    if car_id not in lane_change:
                                        lane_change[car_id] = []
                                    lane_change[car_id].append(t+i)
                                    if car_id2 not in lane_change:
                                        lane_change[car_id2] = []
                                    lane_change[car_id2].append(t+i)

                                    done_set.append(car_id) 
                        elif car_id[0] < 5  and (FV not in lane_change or t not in lane_change[FV]) and right_turning_decision:
                            line = right_turning([px, py], vx)
                            car_num[car_id[0]+1] += 1
                            car_id2 = (car_id[0]+1, car_num[car_id[0]+1])
                            colorset[car_id2] = colorset[car_id]
                            expect_vel[car_id2] = expect_vel[car_id]
                            for i in range(0,turning_time*10+1):
                                seta = math.atan(line[i][2])
                                if t+i < runingtime:
                                    if i < turning_time*10/2:
                                        carSpace[t+i][car_id] = [line[i][0], line[i][1], vx*np.cos(seta), vx*np.sin(seta), 0, 0]
                                    elif i < turning_time*10:
                                        carSpace[t+i][car_id2] = [line[i][0], line[i][1], vx*np.cos(seta), vx*np.sin(seta), 0, 0]
                                    else:
                                        carSpace[t+i][car_id2] = [line[i][0], line[i][1], vx, 0, 0, 0]
                                    if car_id not in decision_making:
                                        decision_making[car_id] = []
                                    decision_making[car_id].append(t+i)
                                    if car_id2 not in decision_making:
                                        decision_making[car_id2] = []
                                    decision_making[car_id2].append(t+i)

                                    if car_id not in lane_change:
                                        lane_change[car_id] = []
                                    lane_change[car_id].append(t+i)
                                    if car_id2 not in lane_change:
                                        lane_change[car_id2] = []
                                    lane_change[car_id2].append(t+i)

                                    done_set.append(car_id) 
                        else:
                            px = px + (vx + vx + ax/10)*1/2*1/10
                            vx = min(max(vx + ax/10, 0), expect_vel[car_id])
                            carSpace[t][car_id] = [px, py, vx, vy, ax, ay]
                    else:                            
                        px = px + (vx + vx + ax/10)*1/2*1/10
                        vx = min(max(vx + ax/10, 0), expect_vel[car_id])
                        carSpace[t][car_id] = [px, py, vx, vy, ax, ay]
    if t % 100 == 0:
        print('IDM+MOBIL: period = %d, time = %d'%(period, t))
outfile = "simulation_velocity_IDM+MOBIL_" + str(period) +".csv"
output = open(outfile, "w+")

for t in range(0,runingtime):
    for car_id in carSpace[t]:
        v = np.hypot(carSpace[t][car_id][2],carSpace[t][car_id][3])
        output.write('%.2f\n'%(v))

# pygame
if 0:
    pygame.init()
    w = 1100*2
    h = 400*2
    def cps(x, y): # change point size
        x = x*4 + 40
        y = -(y*4) + h/2
        return x,y
    
    screen = pygame.display.set_mode((w, h))
    cnt = 0
    for t in range(0,runingtime):
        time.sleep(0.05)            
        screen.fill((255,255,255)) 
        color = [0,0,0]
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

        for car, status in carSpace[t].items():
            # pygame.draw.line(screen,color,item[0],item[1],width)
            colorGroup = [[230,100,100],[100,230,100],[100,100,230]]
            it = CooGet([status[0], status[1], status[3]/(status[2]+0.0001)])
            points = [(cps(it[0][0], it[0][1])),
                      (cps(it[1][0], it[1][1])),
                      (cps(it[2][0], it[2][1])),
                      (cps(it[3][0], it[3][1]))]
            pygame.draw.polygon(screen, colorset[car], points, 0)
        pygame.display.update()    
        pygame.display.flip()    
        # 8 - loop through the events
        for event in pygame.event.get():
            # check if the event is the X button 
            if event.type==pygame.QUIT:
                # if it is quit the game
                pygame.quit() 
                exit(0)


