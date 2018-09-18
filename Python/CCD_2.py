import numpy as np
import matplotlib.pyplot as plt
from CCD_2D import CCD_2D

target = np.array([1,0.5])
n_link = 7

x_data = np.arange(n_link+1)
y_data = np.zeros( (n_link+1,1) )
angle_data = np.zeros((n_link+1,1))
threashold = 0.5

plt.figure(1, figsize=(9,3))
plt.plot(x_data,y_data,'r-')
plt.plot(target[0],target[1],'b*')
plt.show()

# Run CCD Algorithm

output = CCD_2D(x_data,y_data,angle_data,0,target,threashold,10)
new_x_data = output[0]
new_y_data = output[1]
new_angle_data = output[2]

plt.figure(1, figsize=(9,3))
plt.plot(new_x_data,new_y_data,'r-')
plt.plot(target[0],target[1],'b*')
for i in range(0,np.size(x_data)):
    plt.plot(x_data[i],y_data[i],'ob')
plt.show()


