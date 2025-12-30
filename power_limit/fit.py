import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

def func_power(data,k0,k1,k2,k3,k4,k5):
    I,w = data
    return (k0 + k1*I + k2*w + k3*I*w + k4*I**2 + k5*w**2)

array_data = pd.read_csv('m2006data.csv').values
array_power_data = array_data[:,0]
array_I_data = array_data[:,1]
array_w_data = array_data[:,2]

params, covariance = curve_fit(func_power, (array_I_data, array_w_data), array_power_data,p0=[1,1,1,1,1,1])
k0,k1,k2,k3,k4,k5 = params

print('k0:', k0)
print('k1:', k1)
print('k2:', k2)    
print('k3:', k3)
print('k4:', k4)
print('k5:', k5)
