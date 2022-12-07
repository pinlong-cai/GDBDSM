import random, math
import numpy as np
# qr: real flow
# qd: design flow
# n: num of cars
# minTimeHeadWay: time of collision, here is 1 s
# s is random seed
def generateCarFlow(qr, qd, vmin, vmax, runingtime, SampleTime, s):
    random.seed(s)
    minTimeHeadWay = 3/SampleTime
    colorGroup = [[230,100,100],[100,230,100],[100,100,230],
                      [230,200,100],[100,230,200],[200,100,230]]
    Hw = []
    lampda = qr/qd
    hwp = 0
    CarInfo = []
    cnt = 0
    while(1): 
        # generate arrival time
        temp = int(-1/lampda*math.log(random.random())/SampleTime)
        if temp < minTimeHeadWay and len(CarInfo) > 0:
            temp = minTimeHeadWay
        hw = hwp+temp
        if hw > runingtime:
            break
        # info: lane, arrival time, direction, car shape
        # second info is vehicle type
        v = round(random.random()*(vmax-vmin),2)+vmin
        # v = max(min(np.random.randn()*3+(vmax+vmin)/2,vmax),vmin)
        c = colorGroup[random.randint(0,5)]
        s = random.randint(0,4)
        CarInfo.append([cnt, hw, v, c, s])
        hwp = hw 
        cnt += 1
    return CarInfo

