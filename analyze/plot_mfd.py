# plot MFD

import matplotlib.pyplot as plt
import os, pdb
import numpy as np 

files = os.listdir("../model/GDM/result")
Q = []
K = []
for file in files:
    print(file)
    url_r = "../model/GDM/result/" + file
    infile = open(url_r)    
    text_line = infile.readline()
    car_list = {}
    # total simulation time
    base_t = 994
    desity = 0
    flow_dic = {}
    # initial arrival time for calculation
    cut_t = 500

    while(len(text_line) > 1):
        sub = text_line.strip('\n').split(',')
        t, lane, id, pos_x, pos_y, vel = sub[0:6]
        car_id = (lane,id)

        # calculate the density in the last simulation time
        if int(t) == base_t:
            desity += 1
        elif int(t) > base_t:
            break        
        if car_id not in car_list:
            car_list[car_id] = int(t) # record the arrival time
        # calculate the flow
        # init arrival time is over 50 s, and pos is over 450
        if car_id in car_list and car_list[car_id] > cut_t and float(pos_x) > 450:
            flow_dic[car_id] = 1
        text_line = infile.readline()

    # summary the flow
    flow = sum([flow_dic[i] for i in flow_dic])
    infile.close()
    # calculate the desity and flow per hour and per lane
    desity = desity/503*1000 / 7
    flow = flow/(base_t - cut_t)*36000 / 7
    K.append(desity)
    Q.append(flow)


x = np.arange(0,max(K),0.1)
z1 = np.polyfit(K, Q, 3)
p1 = np.poly1d(z1)
pp1=p1(x)

plt.scatter(K, Q, color='red', marker='*',label=u"Simulated traffc states by GDBDSM")
plt.plot(x,pp1,color='#1A5599',linestyle='-',label=u"MFD fitting curve") 

plt.legend(loc='lower right')

plt.xlabel('Density(veh/km/lane)')
plt.ylabel('Flow(veh/h/lane)')
plt.show()


