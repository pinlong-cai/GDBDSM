# comparison: average velocity per minute


import pickle
import matplotlib.pyplot as plt
from scipy import rand
import pdb
import numpy as np

period = 1

pkl_url = '../model/GDM/pkl_period_' + str(period) + '/'
with open(pkl_url+'carSpace.pickle', 'rb') as file:
    carSpace = pickle.load(file)

def read_data(url):        
    infile = open(url)
    data = []
    text_line = infile.readline()
    while(len(text_line)>0):        
        sub = text_line.strip('\n').split(',')
        v = [float(sub[1]),float(sub[0])]
        data.append(v)
        text_line = infile.readline()
    return data

data_i80 = read_data('../I80_data_process/traffic_para/vel_sum_' + str(period) +'.csv')
data_i80.sort(key=lambda data_i80: data_i80[0])

data = []
for t in carSpace:
    for car in carSpace[t]:
        data.append([t,carSpace[t][car][2]])
        

print(np.mean([v[1] for v in data_i80]))
print(np.mean([v[1] for v in data]))
print(np.std([v[1] for v in data_i80]))
print(np.std([v[1] for v in data]))


v = {}
v_r = {}
cnt = 0
cnt_r = 0
dx = 10
x_range = range(dx, 9000+dx, dx)

data = {}
for i in x_range:
    data[i] = [0,0]
for it in data_i80:
    t = int(it[0]*10)
    if t in x_range:
        data[t][0] += it[1]
        data[t][1] += 1

for i in x_range:
    if data[i][1] > 0:  
        data[i] = round(data[i][0]/data[i][1], 3)
    else:
        data[i] = 0



for t in x_range:
    v[t] = [0,0]
    v_r[t] = [0,0]
    while(cnt in carSpace and cnt < 9000):
        for car in carSpace[cnt]:
            v[t][0] += carSpace[cnt][car][2]
            v[t][1] += 1
        cnt += 1
        if cnt+1 > t:
            if v[t][1] > 0:
                v[t] = round(v[t][0]/v[t][1], 3)
            else:
                v[t] = 0
            break
    while(1):
        v_r[t][0] += data_i80[cnt_r][1]
        v_r[t][1] += 1
        cnt_r += 1
        if data_i80[cnt_r][0] * 10 > t:
            if v_r[t][1] > 0:
                v_r[t] = round(v_r[t][0]/v_r[t][1], 3)
            else:
                v_r[t] = -1
            break    
        

v_list = [v[i] for i in v]
v_list_r = [data[i] for i in data]


x_range = [x/10 for x in x_range]

plt.plot(x_range, v_list_r, color = 'b', alpha = 0.5)
plt.plot(x_range, v_list, color = 'r', alpha = 0.5)
plt.ylabel('Average velocity (m/s)')
plt.xlabel('Simulation time (minutes)')
plt.legend(['Real data', 'GDBM'])
plt.show()