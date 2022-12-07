import numpy as np
from roadInfo import * 
from carflow import *
import pygame, time
from car_behavior import *
import pickle
# info process

# map, straight roads with three lanes
lane_width = 3.75
centerline_0 = -lane_width
centerline_1 = 0
centerline_2 = lane_width

tsps = [[2005, 4, 13, 15, 58, 55.3],[],[]] # time_start_periods

running_time = 10000

period = 1
pkl_url = 'pkl_period_' + str(period) + '/'

with open(pkl_url+'carset.pickle', 'rb') as file:
    car_set = pickle.load(file)

with open(pkl_url+'carSpace.pickle', 'rb') as file:
    carSpace = pickle.load(file)

with open(pkl_url+'expect_tra.pickle', 'rb') as file:
    expect_tra = pickle.load(file)

# pygame
def drawText(screen,text,posx,posy,textHeight=32,fontColor=(0,0,0),backgroudColor=(255,255,255)):
    myfont=pygame.font.Font('freesansbold.ttf',textHeight)
    textSurfaceObj=myfont.render(text, True, fontColor, backgroudColor)
    textRectObj=textSurfaceObj.get_rect()
    textRectObj.center=(posx,posy)
    screen.blit(textSurfaceObj,textRectObj.center)
if 1:
    pygame.init()
    w = 1100*2
    h = 400*2
    def cps(x, y): # change point size
        x = x*4 + 40
        y = -(y*4) + h/2
        return x,y

    screen = pygame.display.set_mode((w, h))
    cnt = 0
    for t in range(0,running_time-10):
        time.sleep(0.08)            
        screen.fill((85,87,83)) 
        color = [240,240,240]

        # show time 
        tsp = tsps[period-1]
        # yyyy-mm-dd，hr:mi:se
        se_sum = tsp[3] * 3600 + tsp[4] * 60 + tsp[5] + t/10
        se = '%2.1f'%(round(se_sum%60 ,1))
        if len(se) == 3:
            se = '0' + se
        mi = '%02d'%(int((se_sum%3600)//60))
        hr = '%02d'%(int(se_sum//3600))

        time_show='Time: ' + str(tsp[0])+'-'+str(tsp[1])+'-'+str(tsp[2])+', '+hr+':'+mi+':'+se
        UTC = '(UTC-7)'

        drawText(screen, time_show , 100, 100 ,textHeight=30,fontColor=(240,240,240),backgroudColor=None)
        drawText(screen, UTC , 515, 100 ,textHeight=30,fontColor=(240,240,240),backgroudColor=None)

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
        # draw road
        width = 1
        num_sec = 101
        for line in dotline:        
            for dot in range(0,num_sec):
                if dot % 2 == 0:
                    point0_x, point0_y = (1-dot/num_sec)*line[0][0] + (dot/num_sec)*line[1][0], (1-dot/num_sec)*line[0][1] + (dot/num_sec)*line[1][1]
                    point1_x, point1_y = (1-(dot+1)/num_sec)*line[0][0] + ((dot+1)/num_sec)*line[1][0], (1-(dot+1)/num_sec)*line[0][1] + ((dot+1)/num_sec)*line[1][1]
                    pygame.draw.line(screen,color,cps(point0_x, point0_y),cps(point1_x, point1_y),width)           
        
        # draw vehicle
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
            # drawText(screen,str(car[0])+','+str(car[1]),posx-8,posy-5,textHeight=16,fontColor=(0,0,0),backgroudColor=None)
        pygame.display.update()    
        pygame.display.flip()    
        # 8 - loop through the events
        for event in pygame.event.get():
            # check if the event is the X button 
            if event.type==pygame.QUIT:
                # if it is quit the game
                pygame.quit() 
                exit(0)
