#################################################
# Pioneer P3DX - Sensors and Motors             #
# (c)2023 Djoko Purwanto, djoko@ee.its.ac.id    #
# remake Krisna Pramudya Dharma - 6022221048    #
#################################################

import sim
import sys
import numpy as np
import keyboard
import time
import lib_code as p3dx

# # # # # # # # # # # # # # # # #
# ======== Run Program ======== #
# # # # # # # # # # # # # # # # #

def connect():
    sim.simxFinish(-1) # just in case, close all opened connections

    clientID=sim.simxStart('127.0.0.1',1998,True,True,5000,5) # Connect to CoppeliaSim

    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')

    return clientID

def getObjectHandle(clientID, objectName):
    return sim.simxGetObjectHandle(clientID, objectName, sim.simx_opmode_blocking)[1]

def getObjectPosition(clientID, objectHandle):
    _, position = sim.simxGetObjectPosition(clientID, objectHandle,-1,sim.simx_opmode_oneshot)
    _, orientation = sim.simxGetObjectOrientation(clientID, objectHandle,-1, sim.simx_opmode_oneshot)   
    ret_pos = np.array([position[0], position[1], orientation[2]])

    return ret_pos

def calculateMotion(velo, theta):
    # Calculate the error
    error = np.array([velo, theta])

    # Physical parameters
    WheelRadius = 0.0975 # 0.195 / 2.0
    WidthBody = 0.381

    # Matrix Forward Kinematics
    M_FK = np.array([[WheelRadius/2,                     WheelRadius/2],
                     [WheelRadius/(2*WidthBody),        -WheelRadius/(2*WidthBody)           ]
                    ])
    
    # Matrix Inverse M_FK
    M_I = np.linalg.inv(M_FK)

    vel = np.dot(M_I, error)
    return vel

# ======= Main  Program ======= #

print('Main Program Started')

# ===== Connect to CoppeliaSim Simulator ===== #
clientID = connect()

# ===== Get Disc Handle ===== #
discHandle = getObjectHandle(clientID, '/Disc')

# ===== Get Sensor Handle ===== #
sensorHandle = p3dx.getSensorsHandle(clientID)
motorHandle = p3dx.getMotorHandle(clientID)

# ===== Initial Parameter ===== #
time_prev = time.time()
MaxSpeed = [1.6, 1.6]
_WheelVel = [0.0, 0.0]

# ======= Loop Program ======= #
while True:
    # ======= Routine Run Every 20 ms or 50 ms ======= #
    if(time.time() - time_prev > 0.02):
        time_prev = time.time()

        # Get Sensor 
        sensorV = p3dx.getDistance(clientID, sensorHandle)

        # Calculate Distance and Theta
        if keyboard.is_pressed('l'):
            velocity = 0.0
            theta = 0.2
        elif keyboard.is_pressed('r'):
            velocity = 0.0
            theta = -0.2
        elif keyboard.is_pressed('f'):
            velocity = 0.4
            theta = 0.0
        elif keyboard.is_pressed('b'):
            velocity = -0.4
            theta = 0.0
        else:
            velocity = 0.0
            theta = 0.0

        # Calculate Velocity Robot to Target (Disc)
        WheelVel = calculateMotion(velocity, theta)

        # Limit Speed
        for i in range(2):
            if WheelVel[i] > MaxSpeed[i]:
                WheelVel[i] = MaxSpeed[i]
            elif WheelVel[i] < -MaxSpeed[i]:
                WheelVel[i] = -MaxSpeed[i]

        # Acceleration and Deceleration the Wheel Velocity
        _WheelVel[0] = _WheelVel[0] * 0.85 + WheelVel[0] * 0.15
        _WheelVel[1] = _WheelVel[1] * 0.85 + WheelVel[1] * 0.15

        print(" %.2f %.2f || %.2f %.2f " % (velocity, theta,  _WheelVel[0], _WheelVel[1]))

        # Send Speed to CoppeliaSim Simulator
        p3dx.setWheelVel(clientID, motorHandle, _WheelVel)

    # Press esc to Quit
    if keyboard.is_pressed('esc'):    
        p3dx.setWheelVel(clientID, motorHandle, [0.0, 0.0])
        #? Stop the simulation
        sim.simxFinish
        break
    
p3dx.setWheelVel(clientID, motorHandle, [0.0, 0.0])
# Stop the Simulation
sim.simxFinish
print('Main Program Ended')



