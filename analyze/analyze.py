# make comparisons: real data, IDM+MoBIL, GDBM
# plot velocity distribution curves
# include three periods of data 


import matplotlib.pyplot as plt
from distfit import distfit
import numpy as np
from numpy.core.function_base import linspace
import scipy.stats
from scipy.stats import norm 

def read_data(url):        
    infile = open(url)
    data = []
    text_line = infile.readline()
    while(len(text_line)>0):        
        sub = text_line.strip('\n').split(',')
        v = float(sub[0])
        data.append(v)
        text_line = infile.readline()
    data = data[len(data)//2::] # abandon a half of data, warming up
    return data
def getDistPoint(x, para):
    [u, sig] = para
    y_cdf = [norm.cdf(i, u, sig) for i in x] 
    y_pdf = [norm.pdf(i, u, sig) for i in x] 
    # y_sig = np.exp(-(x - u) ** 2 / (2 * sig ** 2)) / (np.sqrt(2 * np.pi) * sig)
    return y_cdf,y_pdf

def normfit(data):
    data = np.array(data)
    dist_1 = distfit(distr=['norm'])
    #dist = distfit(distr='popular')
    dist_1.fit_transform(data)
    mu_1, sig_1 = dist_1.model['loc'], dist_1.model['scale']
    return [mu_1, sig_1]

def KL_divergence(p,q):
    return scipy.stats.entropy(p, q, base=2)
    # np.sum(p*np.log(p/q)) 


plt.figure(figsize=(10, 10))
x = linspace(0,20,100)
for period in range(1,4):

    data_i80 = read_data('../I80_data_process/traffic_para/vel_sum_' + str(period) +'.csv')
    data_IDM = read_data('../model/IDM+MOBIL/velocity_sum/simulation_velocity_IDM+MOBIL_' + str(period) +'.csv')
    data_GDBM = read_data('../model/GDM/velocity_sum/simulation_velocity_GDM_' + str(period) +'.csv')
    plt.subplot(3,1,period)
    Real_data = getDistPoint(x,normfit(data_i80))
    Baseline = getDistPoint(x,normfit(data_IDM))
    GDBM = getDistPoint(x,normfit(data_GDBM))
    print(KL_divergence(Real_data[0],Baseline[0]))
    print(KL_divergence(Real_data[0],GDBM[0]))
    plt.plot(x,Real_data[1],'-*')
    plt.plot(x,GDBM[1],'-o')
    plt.plot(x,Baseline[1],'-x')
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.ylabel('Probility', fontsize=14)
    plt.xlabel('Velocity (m/s)', fontsize=14)
    plt.legend(['Real data', 'GDBDSM', 'Baseline'], fontsize=14)
plt.show()


