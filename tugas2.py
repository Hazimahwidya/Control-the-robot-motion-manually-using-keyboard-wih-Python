import sim
import sys
import time
import keyboard
import numpy as np

def connectSimulator():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1: 
        print('Connected to remote API server.')
    else:
        print('Connection unsuccesful, program ended.')
        sys.exit()
    return clientID

def getSensorHandle(clientID):
    isNewCoppeliaSim = True
    sensorsHandle = np.array([])
    for i in range(16):
        if(isNewCoppeliaSim):
            sensorHandle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', sim.simx_opmode_blocking)[1]
        else:
            sensorHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), sim.simx_opmode_blocking)[1]
        _, _, _, _, _ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle = np.append(sensorsHandle,sensorHandle)
        sensorsHandle = np.int32(sensorsHandle)
    return sensorsHandle

def getDistance(clientID, sensorsHandle):
    distances = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, sensorsHandle[i], sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 2.0
        distances = np.append(distances,distance)
    return distances

def getMotorHandle(clientID):
    motorLeftHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)[1]
    motorRightHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1]
    motorsHandle = (motorLeftHandle, motorRightHandle)
    return motorsHandle

def setRobotMotion(clientID, motorsHandle, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[1], veloCmd[1], sim.simx_opmode_oneshot)

def invers_kinematik(v,w):
    theta = np.dot(np.linalg.inv([[r/2, r/2],[r/2*l, -r/2*l]]),[v,w])
    return theta[0], theta[1]

print('Program started')
client_id = connectSimulator()
sensors_handle = getSensorHandle(client_id)
motors_handle = getMotorHandle(client_id)

velo_init = (0.0, 0.0)

samp_time, n = 0.1, 1.0

r = 0.195/2
l = 0.381/2

theta = (0.0, 0.0)

time_start = time.time()
while (True):
    if keyboard.is_pressed('w'):
        v = 0.25
        w = 0
        theta = invers_kinematik(v,w)
        setRobotMotion(client_id, motors_handle, theta)
    if keyboard.is_pressed('a'):
        v = 0
        w = -0.25
        theta = invers_kinematik(v,w)
        setRobotMotion(client_id, motors_handle, theta)
    if keyboard.is_pressed('s'):
        v = -0.25
        w = 0
        theta = invers_kinematik(v,w)
        setRobotMotion(client_id, motors_handle, theta)
    if keyboard.is_pressed('d'):
        v = 0
        w = 0.25
        theta = invers_kinematik(v,w)
        setRobotMotion(client_id, motors_handle, theta)

    t_now = time.time()-time_start
    if t_now >= samp_time*n:
        object_distances = getDistance(client_id, sensors_handle)
        front_left_distance = object_distances[2]
        front_right_distance = object_distances[3]
        motors_velo = velo_init
        setRobotMotion(client_id, motors_handle, motors_velo)
        n += 1.0
        print('t = ', round(t_now, 2), 'front side of object distance = ', front_left_distance, '(L),', front_right_distance, '(R)')
    if keyboard.is_pressed('esc'):
        break

sim.simxFinish(client_id)
print('Program ended\n')