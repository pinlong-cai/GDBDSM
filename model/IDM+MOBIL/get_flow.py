import random
import numpy as np

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