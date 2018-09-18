import math
import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import pdb

def CCD_2D( x_data, y_data, angle_data, constraints, goal, threashold, max_iterations):
    "This does stuff"
    
    n_link = x_data.size-1

    # reshape all matrices
    x_data = x_data.reshape(n_link+1,1)
    y_data = y_data.reshape(n_link+1,1)
    angle_data = angle_data.reshape(n_link+1,1)
    goal = goal.reshape(2,1)

    # cast all matrices to float
    x_data = x_data.astype(float)

    jointAngle = angle_data
    output_angles = angle_data
    # Initialization of Data
    # Set up plot (DEBUG)
    plt.axis([-1, 10, -1, 7])
    plt.ion()
    plt.show()
    # plot data (DEBUG)
    line, = plt.plot(x_data,y_data,'r-')
    plt.plot(goal[0],goal[1],'b*')
    plt.draw()
    plt.pause(0.001)
    raw_input("Press [Enter] to continue")
    
    error = np.linalg.norm(np.array([(x_data[n_link]),(y_data[n_link])])-goal)

    #pdb.set_trace()

    n_iterations = 0

    while (error > threashold and n_iterations < max_iterations):
        active_joint = n_link + 1
        while active_joint > 1:
            # CCD Algorithm
            # End Effector Position
            pe = np.array([x_data[n_link],y_data[n_link]])
            # Current Joint Position
            pc = np.array([x_data[active_joint-2],y_data[active_joint-2]])
            # vector from end effector to current joint
            a = (pe-pc)/np.linalg.norm(pe-pc)
            a = a.reshape(1,2)
            # vector from current joint to target
            b = (goal-pc)/np.linalg.norm(goal-pc)
            # rotation angle for current joint to minimize b
            theta = math.acos(np.dot(a,b))
            print("a")
            print(a)
            print("b")
            print(b)
            print("Theta")
            print(math.degrees(theta))
            #pdb.set_trace() # DEBUG
            # check for rotation direction
            b = b.reshape(1,2)
            direction = np.cross(np.array([a[0,0],a[0,1],0]),np.array([b[0,0],b[0,1],0]))
            if direction[2] < 0:
                theta = -theta
            # joint constraints
            if theta > (pi/3):
                theta = (pi/3)
            elif theta < -(pi/3):
                theta = -(pi/3)
            angle_data[active_joint-1]=theta
            jointAngle[active_joint-1]=theta

            # Perform Rotation

            # for first joint
            i = active_joint
            # calculate rotation matrix
            R = np.array([(math.cos(angle_data[i-1]),-math.sin(angle_data[i-1])),(math.sin(angle_data[i-1]),math.cos(angle_data[i-1]))])

            pdb.set_trace() # DEBUG
            while i<=n_link+1:
                #pdb.set_trace() # DEBUG
                # rotate each joint around active joint
                temp = R.dot(np.array([x_data[i-1]-x_data[active_joint-2],y_data[i-1]-y_data[active_joint-2]])) + np.array([x_data[active_joint-2],y_data[active_joint-2]])
                x_data[i-1] = temp[0,0]
                y_data[i-1] = temp[1,0]
                angle_data[i-1] = angle_data[i-1] + angle_data[i-2]
                print("temp")
                print(temp)
                print("x_data")
                print(x_data)
                print("y_data")
                print(y_data)
                print("angle_data")
                print(angle_data)

                # plot data (DEBUG)
                line.set_xdata(x_data)
                line.set_ydata(y_data)
                plt.draw()
                plt.pause(0.001)
                raw_input("Press [Enter] to continue")

                #pdb.set_trace() # DEBUG
                i = i + 1
            # advance iteration
            active_joint = active_joint - 1
        # calculate error
        error = np.linalg.norm(np.array([(x_data[n_link]),(y_data[n_link])])-goal)
        # update output angles with current iteration deltas
        output_angles = output_angles + jointAngle
        # add to iteration count
        n_iterations = n_iterations + 1
    return (x_data,y_data,angle_data)

