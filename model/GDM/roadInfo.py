
import numpy as np
import math



carshape = [4,2]
hl,hw = carshape[0]/2, carshape[1]/2
margin = 0
turning_time = 2
len_lane = 503
wl = 3.75 # width_lane
predict_time = 2
# rt = 1*10 # reaction time
init_pos = [[0,wl*5.5],[0,wl*4.5],[0,wl*3.5],
            [0,wl*2.5],[0,wl*1.5],[0,wl*0.5],
            [100,-3-wl*0.5]]

lane_y = []
for i in range(len(init_pos)-1):
    lane_y.append(init_pos[i][1])

def distance(pos0, pos1):
    x0, y0 = pos0[0], pos0[1]
    x1, y1 = pos1[0], pos1[1]
    return np.sqrt((x0-x1)**2+(y0-y1)**2)

def CooGet(para, margin = margin):
    hwm = hw + margin
    hlm = hl + margin
    x, y, k = para
    seta = math.atan(k)
    seta0 = math.atan(hwm/hlm)
    dx1 = round(np.sqrt(hwm**2+hlm**2) * math.cos(seta0+seta),3)
    dy1 = round(np.sqrt(hwm**2+hlm**2) * math.sin(seta0+seta),3)
    dx2 = round(np.sqrt(hwm**2+hlm**2) * math.cos(seta0-seta),3)
    dy2 = round(np.sqrt(hwm**2+hlm**2) * math.sin(seta0-seta),3)
    Pa = [round(x-dx2,3),round(y+dy2,3)]
    Pb = [round(x-dx1,3),round(y-dy1,3)]
    Pc = [round(x+dx2,3),round(y-dy2,3)]
    Pd = [round(x+dx1,3),round(y+dy1,3)]
    return [Pa, Pb, Pc, Pd]

def B(n,i,t):    
    return math.factorial(n)/(math.factorial(i)*math.factorial(n-i)) * t**i * (1-t)**(n-i)
def Bezier(p,n,c,T):
    res = {}
    k = 0
    for t in np.arange(0, T+c, c):
        x, y = 0, 0
        for i in range(n+1):
            x += B(n,i,t/T) * p[i][0] 
            y += B(n,i,t/T) * p[i][1] 
        if t>0 :
            k = (y - y_p)/(x - x_p+0.0001)
        t = round(t, 2)
        x, y, k = round(x, 3), round(y, 3), round(k, 3)
        res[t] = [x,y,k]
        x_p, y_p = x, y
    c2 = c+c
    res[0][2] = round(res[c][2] * 3 - res[c2][2],3) # 端点斜率优化
    return res

def left_turning(ori_pos, vel):
    x, y = ori_pos[0], ori_pos[1]
    del_x = vel * turning_time/4
    Bp1 = [(x,y),(x+del_x,y),(x+2*del_x,y+wl/2),(x+3*del_x,y+wl),(x+4*del_x,y+wl)]
    LC = Bezier(Bp1, len(Bp1)-1, 1, turning_time*10) # 50分段
    return LC 

def right_turning(ori_pos, vel):
    x, y = ori_pos[0], ori_pos[1]
    del_x = vel * turning_time/4
    Bp1 = [(x,y),(x+del_x,y),(x+2*del_x,y-wl/2),(x+3*del_x,y-wl),(x+4*del_x,y-wl)]
    RC = Bezier(Bp1, len(Bp1)-1, 1, turning_time*10)
    return RC 

def straight_going(ori_pos, vel):
    x, y = ori_pos[0], ori_pos[1]
    SC = {0:[x,y,0]}
    for t in np.arange(1, predict_time*10+1, 1):
        SC[t] = [round(x+vel*t/10,4),y,0]
    return SC 

def onramp(ori_pos, vel):
    x, y = ori_pos[0], ori_pos[1]
    seta = np.arctan(0.03)
    SC = {0:[x,y,0]}
    for t in np.arange(1, turning_time*10+1, 1):
        s = vel*t/10
        SC[t] = [round(x+s*np.cos(seta),4),round(y+np.sin(seta)),0.03]
    return SC 

def confluence(ori_pos, vel):
    x, y = ori_pos[0], ori_pos[1]
    Bp1 = [(x,y),(250,y+wl),(300,y+wl)]
    confluence_time = (300-x)/vel
    CC = Bezier(Bp1, len(Bp1)-1, 1, confluence_time*10)
    return CC 

    
    
