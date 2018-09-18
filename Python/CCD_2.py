import numpy as np
import matplotlib.pyplot as plt
from CCD_2D import CCD_2D

# Set up initial data
target = np.array([1,0.5])
n_link = 7
x_data = np.arange(n_link+1)
y_data = np.zeros( (n_link+1,1) )
angle_data = np.zeros((n_link+1,1))
threashold = 0.5

# Set up plot
plt.axis([-1, 10, -1, 7])
plt.ion()
plt.show()
# Plot Data
line, = plt.plot(x_data,y_data,'r-')
plt.plot(target[0],target[1],'b*')
plt.draw()
plt.pause(0.001)
raw_input("Press [enter] to continue")

# Run CCD Algorithm

output = CCD_2D(x_data,y_data,angle_data,0,target,threashold,10)
new_x_data = output[0]
new_y_data = output[1]
new_angle_data = output[2]

# Update Plot with new data
line.set_xdata(new_x_data)
line.set_ydata(new_y_data)
plt.draw()
plt.pause(0.001)
for i in range(0,np.size(x_data)):
    plt.plot(new_x_data[i],new_y_data[i],'ob')
plt.show()
raw_input("Press [enter] to continue")

