import math
import numpy as np
from numpy import pi
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
    
    error = np.linalg.norm(np.array([(x_data[n_link]),(y_data[n_link])])-goal)

    #pdb.set_trace()

    n_iterations = 0

    while (error > threashold and n_iterations < max_iterations):
        active_joint = n_link
        while active_joint > 0:
            # CCD Algorithm
            # End Effector Position
            pe = np.array([x_data[n_link],y_data[n_link]])
            # Current Joint Position
            pc = np.array([x_data[active_joint-1],y_data[active_joint-1]])
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
            pdb.set_trace() # DEBUG
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
            angle_data[active_joint]=theta
            jointAngle[active_joint]=theta

            # Perform Rotation

            # for first joint
            i = active_joint
            # calculate rotation matrix
            R = np.array([(math.cos(angle_data[i]),-math.sin(angle_data[i])),(math.sin(angle_data[i]),math.cos(angle_data[i]))])

            pdb.set_trace() # DEBUG
            while i<=n_link:
                #pdb.set_trace() # DEBUG
                # rotate each joint around active joint
                temp = R.dot(np.array([x_data[i]-x_data[i-1],y_data[i]-y_data[i-1]])) + np.array([x_data[active_joint-1],y_data[active_joint-1]])
                x_data[i] = temp[0,0]
                y_data[i] = temp[1,0]
                angle_data[i] = angle_data[i] + angle_data[i]
                print("temp")
                print(temp)
                print("x_data")
                print(x_data)
                print("y_data")
                print(y_data)
                print("angle_data")
                print(angle_data)
                pdb.set_trace() # DEBUG
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

