from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math

# Getting data with Distance Sensor functions
def inverseKinematics(v, omega, l):
    r = 20.5 / 1000
    phi1 = (v + ((l * omega) / 2)) / r
    phi2 = (v - ((l * omega) / 2)) / r
    
    wheelsVelocity = np.array([phi1,
                               phi2])
    return wheelsVelocity

def forwardKinematics(kesiI, v1, v2, t, l):
    x, y, teta = kesiI
    Vx = (v1 + v2) / 2
    Vy = 0
    omega = (v1 - v2) / l
    kesiDotR = np.array([[Vx],
                         [Vy],
                         [omega]])
    RInverse = np.array([[math.cos(-teta), math.sin(-teta), 0],
                  [-math.sin(-teta), math.cos(-teta), 0],
                  [0, 0, 1]])
    kesiDotI = RInverse.dot(kesiDotR)
    newKesiI = np.array([x + t * kesiDotI[0],
                         y + t * kesiDotI[1],
                         teta + t * kesiDotI[2]])
    return newKesiI

# Get robot's heading in degree based on compass values
def getRobotHeading(compassValue):
    rad = math.atan2(compassValue[1], compassValue[0])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    
    heading = 360 - bearing
    return heading

def tableToReal(lookupTable, output):
    measured = lookupTable[:, 1]
    noMeasured = len(measured)
    index = 0

    for i in range(noMeasured):
        if measured[i] <= output:
            index = i
            break

    if index == 0:
        if output <= measured[noMeasured - 1]:
            return lookupTable[:, 0][noMeasured - 1]
        return 0

    firstDotY = measured[index - 1]
    firstDotX = lookupTable[:, 0][index - 1]
    secondDotY = measured[index]
    secondDotX = lookupTable[:, 0][index]
    slope = (firstDotY - secondDotY) / (firstDotX - secondDotX)
    intercept = firstDotY - (slope * firstDotX)
    realDistance = (output - intercept) / slope
    return realDistance

def spin90(toLeft, steps = 49):
    for i in range(steps):
        if toLeft:
            leftMotor.setVelocity(-WHEELS_VELOCITY)
            rightMotor.setVelocity(WHEELS_VELOCITY)
        else:
            leftMotor.setVelocity(WHEELS_VELOCITY)
            rightMotor.setVelocity(-WHEELS_VELOCITY)
        robot.step(TIME_STEP)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    value2 = distanceSensor2.getValue()
    return tableToReal(lookupTable, value2)

def toWallDegree():
    initialHeading = getRobotHeading(compass.getValues())
    if 0 <= initialHeading <= 45:
        degree = 0.0
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(-WHEELS_VELOCITY)
    elif 315 <= initialHeading <= 360:
        degree = 360.0
        leftMotor.setVelocity(-WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
    elif 45 < initialHeading <= 90:
        degree = 90.0
        leftMotor.setVelocity(-WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
    elif 90 < initialHeading <= 135:
        degree = 90.0
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(-WHEELS_VELOCITY)
    elif 135 <= initialHeading <= 180:
        degree = 180.0
        leftMotor.setVelocity(-WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
    elif 180 < initialHeading <= 225:
        degree = 180.0
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(-WHEELS_VELOCITY)
    elif 225 < initialHeading <= 270:
        degree = 270.0
        leftMotor.setVelocity(-WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
    elif 270 < initialHeading < 315:
        degree = 270.0
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(-WHEELS_VELOCITY)
    while not(abs(getRobotHeading(compass.getValues()) - degree)) < 3.0:
        robot.step(TIME_STEP)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

def rotateToGoal(finalDestination):
    x, y, _ = gps.getValues()
    angle = math.atan2(finalDestination[1] - y, finalDestination[0] - x) * 180 / math.pi
    initialHeading = getRobotHeading(compass.getValues())
    if initialHeading > 180.0:
        initialHeading -= 360.0
    if angle > initialHeading:
        leftMotor.setVelocity(-WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
    else:
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(-WHEELS_VELOCITY)
    lastHeading = initialHeading
    while not(abs(lastHeading - angle) < 3.0):
        robot.step(TIME_STEP)
        lastHeading = getRobotHeading(compass.getValues())
        if lastHeading > 180.0:
            lastHeading -= 360.0
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

def distanceBetweenNodes(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

def getLineEquation(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - (slope * x1)
    return slope, intercept

def isOnLine(node, slope, intercept):
    x, y = node
    return abs(y - (slope * x + intercept)) < 0.02

# Create an instance of robot
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

WHEELS_VELOCITY = 3

# Load Devices such as sensors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
distanceSensor2 = robot.getDevice('ps2')
distanceSensor6 = robot.getDevice('ps6')

# Enables the devices
distanceSensor2.enable(TIME_STEP)
distanceSensor6.enable(TIME_STEP)
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)

robot.step(TIME_STEP) # take some dummy steps in environment for safe initialization

initialLocation = gps.getValues()
x, y, _ = initialLocation

# Set the motors to rotate for ever
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(WHEELS_VELOCITY)
rightMotor.setVelocity(WHEELS_VELOCITY)

# General parameters
finalDestination = (0, -0.5)
lookupTable = distanceSensor2.getLookupTable()
lookupTable = np.array(lookupTable)
lookupTable = lookupTable.reshape(int(lookupTable.size/3), 3)
value2 = distanceSensor2.getValue()
lastDistance2 = tableToReal(lookupTable, value2)
xValues = []
yValues = []
lastBreakPoint = (x, y)
slope, intercept = getLineEquation((x, y), finalDestination)
mode = 'turnToGoal'

while abs(x - finalDestination[0]) > 0.1 or abs(y - finalDestination[1]) > 0.1:
    if mode == 'turnToGoal':
        rotateToGoal(finalDestination)
        mode = 'moveTowardsGoal'
    elif mode == 'moveTowardsGoal':
        value6 = distanceSensor6.getValue()
        realDistance6 = tableToReal(lookupTable, value6)
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
        if realDistance6 < 0.04:
            mode = 'spin90True'
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
        robot.step(TIME_STEP)
    elif mode == 'spin90True':
        lastDistance2 = spin90(True)
        mode = 'followWall'
    elif mode == 'spin90False':
        lastDistance2 = spin90(False)
        mode = 'followWall'
    elif mode == 'followWall':
        leftMotor.setVelocity(WHEELS_VELOCITY)
        rightMotor.setVelocity(WHEELS_VELOCITY)
        robot.step(TIME_STEP)
        value2 = distanceSensor2.getValue()
        realDistance2 = tableToReal(lookupTable, value2)
        value6 = distanceSensor6.getValue()
        realDistance6 = tableToReal(lookupTable, value6)
        if realDistance6 < 0.04:
            mode = 'spin90True'
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            continue
        if realDistance2 < 0.04:
            if realDistance2 < 0.01:
                spin90(True, 5)
                leftMotor.setVelocity(WHEELS_VELOCITY)
                rightMotor.setVelocity(WHEELS_VELOCITY)
                for i in range(5):
                    robot.step(TIME_STEP)
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
            mode = 'toWallDegree'
        elif realDistance2 > 3 * lastDistance2:
            mode = 'spin90False'
            for i in range(60):
                value6 = distanceSensor6.getValue()
                realDistance6 = tableToReal(lookupTable, value6)
                if realDistance6 < 0.04:
                    mode = 'spin90True'
                    break
                if i < 8:
                    value2 = distanceSensor2.getValue()
                    tempRealDistance2 = tableToReal(lookupTable, value2)
                    if not (tempRealDistance2 > 3 * lastDistance2):
                        mode = 'followWall'
                        break
                robot.step(TIME_STEP)
                x, y, _ = gps.getValues()
                xValues.append(-x)
                yValues.append(-y)
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        lastDistance2 = realDistance2
    elif mode == 'toWallDegree':
        toWallDegree()
        mode = 'followWall'
    xValues.append(-x)
    yValues.append(-y)
    x, y, _ = gps.getValues()
    if isOnLine((x, y), slope, intercept) and distanceBetweenNodes((x, y), finalDestination) < distanceBetweenNodes(lastBreakPoint, finalDestination) and mode == 'followWall':
        lastBreakPoint = (x, y)
        mode = 'turnToGoal'

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

plt.figure()
plt.plot(xValues, yValues)
plt.plot(-initialLocation[0], -initialLocation[1], 'r', marker='*', ms=8)
plt.plot(-finalDestination[0], -finalDestination[1], 'g', marker='*', ms=8)
plt.show()