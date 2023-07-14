############################################################
#   Pioneer P3DX - Manual Control Using Keyboard
#   Hazimah Widyagustin
#   6022221040 
############################################################

import sim
import sys
import time
import keyboard
import numpy as np

def connectSimulator():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1: print('Connected to remote API server.')
    else:
        print('Connection unsuccesful, program ended.')
        sys.exit()
    return clientID

def getSensorHandle(clientID):
    sensorsHandle = np.array([])
    for i in range(16):
        sensorHandle = sim.simxGetObjectHandle(
            clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), sim.simx_opmode_blocking)[1]
        _, _, _, _, _ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle = np.append(sensorsHandle,sensorHandle)
    return sensorsHandle

def getMotorHandle(clientID):
    motorLeftHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)[1]
    motorRightHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1]
    return motorLeftHandle, motorRightHandle

def getDistance(clientID, sensors):
    distances = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, int(sensors[i]), sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 10.0
        distances = np.append(distances,distance)
    return distances

def setRobotMotion(clientID, motors, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motors[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motors[1], veloCmd[1], sim.simx_opmode_oneshot)
    
print('Program started')
client_id = connectSimulator()
motor_left_handle, motor_right_handle = getMotorHandle(client_id)
sensors_handle = getSensorHandle(client_id)
samp_time = 0.1
n = 1
velo_zero = (0,0)
velo_forward = (2,2)
velo_left = (2,0)
velo_backward = (-2,-2)
velo_right = (0,2)
time_start = time.time()
while (True):
    if keyboard.is_pressed('F'):
        setRobotMotion(client_id, (motor_left_handle,motor_right_handle),velo_forward)
    if keyboard.is_pressed('R'):
        setRobotMotion(client_id, (motor_left_handle,motor_right_handle),velo_right)
    if keyboard.is_pressed('B'):
        setRobotMotion(client_id, (motor_left_handle,motor_right_handle),velo_backward)
    if keyboard.is_pressed('L'):
        setRobotMotion(client_id, (motor_left_handle,motor_right_handle),velo_left)
    t_now = time.time()-time_start
    if t_now >= samp_time*n:
        object_distances = getDistance(client_id, sensors_handle)
        setRobotMotion(client_id,(motor_left_handle,motor_right_handle),velo_zero)
        n += 1
        print('t = ', round(t_now, 2), 'front side object distance = ', object_distances[3],object_distances[4])
    if keyboard.is_pressed('esc'):
        setRobotMotion(client_id, (motor_left_handle,motor_right_handle),velo_zero)
        break

sim.simxFinish(client_id)
print('Program ended\n')