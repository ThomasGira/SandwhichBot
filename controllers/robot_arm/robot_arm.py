"""robot_arm controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera,CameraRecognitionObject, Keyboard
import math

import cv2
import numpy as np
from matplotlib import pyplot as plt


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)


# class Camera (Device):
    # def getWidth(self):
    # def getHeight(self)
    # def saveImage(self, filename, quality):
    # def getImage(self):
    # def getImageArray(self):
    # def imageGetRed(image, width, x, y):
    # def imageGetGreen(image, width, x, y):
    # def imageGetBlue(image, width, x, y):
    # def imageGetGray(image, width, x, y):
    # def hasRecognition(self):
    # def recognitionEnable(self, samplingPeriod):
    # def recognitionDisable(self):
    # def getRecognitionSamplingPeriod(self):
    # def getRecognitionNumberOfObjects(self):
    # def getRecognitionObjects(self):
    # def hasRecognitionSegmentation(self):
    # def enableRecognitionSegmentation(self):
    # def disableRecognitionSegmentation(self):
    # def isRecognitionSegmentationEnabled(self):
    # def getRecognitionSegmentationImage(self):
    # def getRecognitionSegmentationImageArray(self):
    # def saveRecognitionSegmentationImage(self, filename, quality):
     
# class CameraRecognitionObject:
    # def get_id(self):
    # def get_position(self):
    # def get_orientation(self):
    # def get_size(self):
    # def get_position_on_image(self):
    # def get_size_on_image(self):
    # def get_number_of_colors(self):
    # def get_colors(self):
    # def get_model(self):
    
camera = robot.getDevice('camera')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()
camera.recognitionEnable(10*timestep)
#camera.enableRecognitionSegmentation()

##############################################################################################


"""
# Opening image
img = cv2.imread("image.jpg")
  
# OpenCV opens images as BRG 
# but we want it as RGB and 
# we also need a grayscale 
# version
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  
# Creates the environment 
# of the picture and shows it
plt.subplot(1, 1, 1)
plt.imshow(img_rgb)
plt.show()

"""

# Get and enable the camera device.



####################################################################################

#Initialize Motors and Sensors
motorList = ['base', 'upperarm', 'forearm', 'wrist', 'rotational_wrist', 'left_gripper', 'right_gripper']
sensorList = ['base_sensor', 'upperarm_sensor', 'forearm_sensor', 'wrist_sensor', 'rotational_wrist_sensor', 'left_gripper_sensor', 'right_gripper_sensor','ts1','ts3']

motors = []
sensors = []
sensorData = []
angularVelocity = .5
pedDetected = False
for motor in motorList:
    motorNode = robot.getDevice(motor)
    motorNode.setVelocity(.5) #Hard code velocity, can be updated later to optimize efficiency and safety
    motorNode.setPosition(0)
    motors.append(motorNode)
for sensor in sensorList:
    sensorNode = robot.getDevice(sensor)
    sensorNode.enable(timestep)
    sensors.append(sensorNode)
#Arm lengths
L1 = .2035 #Base
L2 = .19   #Arm 1
L3 = .139 #Arm 2
L4 = .325
#Base Location
x_base = 2.68
y_base = .6
z_base = 2.8

#Initialize Settings
goal_update = False 
open = False
close = False

#Arm movement function
#Need to implement forwards kinematics so that it can detect if the object is being lifted or lowered
def moveArm(goals,lift):
    x_goal = goals[0]
    y_goal = goals[1]
    z_goal = goals[2]
    #Cylindrical coordinates (X is positive red, Y is negative Blue, Z is positive Green)
    theta = math.atan2(y_goal,x_goal)
    if theta < 0 :
        theta = 2*math.pi + theta
    r = (x_goal**2 + y_goal**2)**.5
    z = z_goal - L1

    y = r - L4
    x = z
    q2 = math.acos(((x**2 + y**2)**.5-L2)/(L3))

    if q2 > 0: #Force the arm to come from above
        q2 = q2
    q1 = math.atan2(y,x) - math.atan2(L3*math.sin(q2),L2 + L3*math.cos(q2))
    q3 = q1 + q2 - math.pi/2
    #Edge cases
    if q1 > 1.74 or q1< -1.74:
        print('Invalid Position: Upperarm')
        return
    if q2 > 2.45 or q2 < -2.45:
        print('Invalid Position: Forearm')
        return
    if q3 > 2.1 or q3 < -2.1:
        print('Invalid Position: Wrist')
        return
    #Normalize motors to lift veritcally
    pose1 = sensors[1].getValue()
    pose2 = sensors[2].getValue()
    pose3 = sensors[3].getValue()
    
    delta1 = abs(pose1+q1)
    delta2 = abs(pose2+q2)
    delta3 = abs(pose3-q3)
    
    if delta1 > delta2 and delta1 > delta3:
        norm1 = delta1/delta1
        norm2 = delta2/delta1
        norm3 = delta3/delta1
        
    if delta2 > delta1 and delta2 > delta3:
        norm1 = delta1/delta2
        norm2 = delta2/delta2
        norm3 = delta3/delta2
        
    if delta3 > delta2 and delta3 > delta1:
        norm1 = delta1/delta3
        norm2 = delta2/delta3
        norm3 = delta3/delta3
        
    v1 = (norm1*angularVelocity)
    v2 = (norm2*angularVelocity)
    v3 = (norm3*angularVelocity)
        
    #Set motor position
    if lift == False: #Lowering
        while abs(sensors[0].getValue() - theta) > .001:
            if safety():
                motors[1].setVelocity(norm1*angularVelocity)
                motors[2].setVelocity(norm2*angularVelocity)
                motors[3].setVelocity(norm3*angularVelocity) 
            motors[0].setPosition(theta)
            robot.step(timestep)
        
        while abs(sensors[1].getValue() + q1 )> .0001 or abs(sensors[2].getValue() + q2) > .0001:
            if safety():
                motors[1].setVelocity(norm1*angularVelocity)
                motors[2].setVelocity(norm2*angularVelocity)
                motors[3].setVelocity(norm3*angularVelocity)            
            motors[1].setPosition(-q1)
            motors[2].setPosition(-q2)
            motors[3].setPosition(q3)
            robot.step(timestep)
    else: #Raising
        while abs(sensors[1].getValue() + q1) > .1 or abs(sensors[2].getValue() + q2) > .1:
            if safety():
                motors[1].setVelocity(norm1*angularVelocity)
                motors[2].setVelocity(norm2*angularVelocity)
                motors[3].setVelocity(norm3*angularVelocity) 
            
            motors[1].setPosition(-q1)
            motors[2].setPosition(-q2)
            motors[3].setPosition(q3)
            robot.step(timestep)
        
        while abs(sensors[0].getValue() - theta) > .01:
            if safety():
                motors[1].setVelocity(norm1*angularVelocity)
                motors[2].setVelocity(norm2*angularVelocity)
                motors[3].setVelocity(norm3*angularVelocity) 
            
            motors[0].setPosition(theta)
            robot.step(timestep)
        
#Open Grippers
#Add functionality to open at set width
def openGrip():
    while sensors[5].getValue() > -1.1999 and sensors[6].getValue() <1.1999:
        safety()
        motors[5].setPosition(-1.2)
        motors[6].setPosition(1.2)
        robot.step(timestep)

def closeGrip():
    while sensors[7].getValue() < 200 or sensors[8].getValue() < 200:
        safety()
        #print(sensors[7].getValue(),sensors[8].getValue())
        motors[5].setPosition(0)
        motors[6].setPosition(0)
        robot.step(timestep)
    motors[5].setPosition(sensors[5].getValue())
    motors[6].setPosition(sensors[6].getValue())
    #print(sensors[7].getValue(),sensors[8].getValue())

def convertCoordinates(wx,wy,wz):
    x_goal = wx - x_base
    y_goal = z_base - wz
    z_goal = wy - y_base
    
    goals = (x_goal,y_goal,z_goal)
    return goals
    
def cameraCoordinates(position):
    x_goal = position[0]
    y_goal = position[1]
    z_goal = position[2]+1.23
    
    goals = (x_goal,y_goal,z_goal)
    return goals
    
def adjustGoals(position,x,y,z):
    x_goal = position[0]+x
    y_goal = position[1]+y
    z_goal = position[2]+z
    
    goals = (x_goal,y_goal,z_goal)
    return(goals)
    

def getObjLocation(object):
    objects = camera.getRecognitionObjects()
    for obj in range(len(objects)):
        #print("Name:",objects[obj].get_model()," Position: ", objects[obj].get_position())
        name = objects[obj].get_model().decode("utf-8")
        pose = objects[obj].get_position()
        if (object in name) == True:
            return cameraCoordinates(pose)
    print('Object not found: ',object)
    return False
def addSauce(lastItem,Item):
    base = lastItem
    itemPose = getObjLocation(Item)
    openGrip()
    moveArm(adjustGoals(itemPose,0,0,.1),False)
    moveArm(itemPose,False)
    closeGrip()
    goHome()
    moveArm(adjustGoals(base,0,0,.2),False)
    sauce()
    goHome()
    moveArm(adjustGoals(itemPose,0,0,.1),False)
    moveArm(itemPose,False)
    openGrip()
    goHome()
    return base
    
def sauce():
    motors[4].setPosition(math.pi)
    while sensors[4].getValue() < 3.1:
        robot.step(timestep)
    motors[4].setPosition(0)
    while sensors[4].getValue() > .1:
        robot.step(timestep)
        
def getPose():
    theta = sensors[0].getValue()
    q1 = -sensors[1].getValue()
    q2 = -sensors[2].getValue()
    q3 = sensors[3].getValue()
    
    z = L2*math.sin(q1) + L3*math.sin(q1+q2) + L4
    r = L2*math.cos(q1) + L3*math.cos(q1+q2) + L1
    
    y = r*math.sin(theta)
    x = r*math.cos(theta)
    
    pose = (x,y,z)
    return pose
    
def goHome():
    goals = (0,-.4,.5)
    moveArm(goals,True)
    
def addToStack(lastItem,Item):
    base = lastItem
    itemPose = getObjLocation(Item)
    openGrip()
    moveArm(adjustGoals(itemPose,0,0,.1),False)
    moveArm(itemPose,False)
    closeGrip()
    clearance(itemPose)
    moveArm(adjustGoals(base,0,0,.04),False)
    openGrip()
    clearance(base)
    return adjustGoals(base,0,0,.04)
    
def makeSandwhich(list):
    topStack = adjustGoals(getObjLocation("plate"),0,0,.02)
    for item in list:
        if item == "Ketchup":
            addSauce(topStack,item)
        else:
            topStack = addToStack(topStack,item)
        
def clearance(pose):
   x = pose[0]
   y = pose[1]
   z = .4
   
   height = (x,y,z)
   moveArm(height,True)
   
def setSpeed(speed):
    for motor in motors:
        motor.setVelocity(speed)
        
def safety():
    safetyCheck = getObjLocation('pedestrian')
    if safetyCheck != False:
        # Stop if within certain distance
        dis = np.sqrt(safetyCheck[0]**2 + safetyCheck[1]**2)
        if dis <= 1:
            # Stop Robot somehow
            setSpeed(0)
            print("Safety distance has been breached by Ped")
            return False
        else: 
            setSpeed(.5)
            return True
print("""
---------------
 Whatchu Want?
 --------------
 """)
print ("""
1.BSLSB
2.BSB
3.BSLKB
4.BKB
""")        
while robot.step(timestep) != -1:  
    key = keyboard.getKey()
    while(keyboard.getKey() != -1): pass
    if key == ord('1'):
        print("You get a BSLSB")
        makeSandwhich(("Bread","Salami","Lettuce","Salami","Bread"))
        print("EET FREEF")
    elif key == ord('2'):
        print("You get a BSB")
        makeSandwhich(("Bread","Salami","Bread"))
        print("EET FREEF")
    elif key == ord('3'):
        print("You get a BSLKB")
        makeSandwhich(("Bread","Salami","Lettuce","Ketchup""Bread"))
        print("EET FREEF")
    elif key == ord('4'):
        print("You get a BKB")
        makeSandwhich(("Bread","Ketchup","Bread"))
        print("EET FREEF")