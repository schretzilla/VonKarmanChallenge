#! /usr/bin/python

from Servo import Servo
import time

class Arm():
    #Servo Indicies
    BasePanIndex = 0
    BaseHomeAngle = 90
    BaseMinAngle = 0
    BaseMaxAngle = 180

    ShoulderIndex = 5
    ShoulderHomeAngle = 90
    ShoulderMinAngle = 0
    ShoulderMaxAngle = 180

    ElbowIndex = 1
    ElbowHomeAngle = 90
    ElbowMinAngle = 0
    ElbowMaxAngle = 180
    
    WristPanIndex = 2
    WristPanHomeAngle = 85 
    WristPanMinAngle = 0
    WristPanMaxAngle = 180

    WristIndex = 3
    WristHomeAngle = 90
    WristMinAngle = 5
    WristMaxAngle = 145
    
    GripperIndex = 4
    GripperMinAngle = 105
    GripperHomeAngle = GripperMinAngle
    GripperMaxAngle = 180

    def __init__(self):
        #Initiate servo objects
        baseServo = Servo(self.BasePanIndex, self.BaseHomeAngle, self.BaseMinAngle, self.BaseMaxAngle)
        shoulderServo = Servo(self.ShoulderIndex, self.ShoulderHomeAngle, self.ShoulderMinAngle, self.ShoulderMaxAngle)
        elbowServo = Servo(self.ElbowIndex, self.ElbowHomeAngle, self.ElbowMinAngle, self.ElbowMaxAngle)
        wristPanServo = Servo(self.WristPanIndex, self.WristPanHomeAngle, self.WristPanMinAngle, self.WristPanMaxAngle)
        wristServo = Servo(self.WristIndex, self.WristHomeAngle, self.WristMinAngle, self.WristMaxAngle)
        gripperServo = Servo(self.GripperIndex, self.GripperHomeAngle, self.GripperMinAngle, self.GripperMaxAngle)

        #Populate servo list
        self.m_servoList = [baseServo, shoulderServo, elbowServo,
        wristPanServo, wristServo, gripperServo]
        #set current servo index
        self.m_currentServoIndex = 0

        #Move to home position
        self.HomePosition()

    #Move to next servo index. Cycle back to zero if on the last servo
    def CycleToNextServo(self):
        if(self.m_currentServoIndex < len(self.m_servoList) - 1):
            self.m_currentServoIndex += 1
        else:
            #Move to beggining of list
            self.m_currentServoIndex = 0


    #Move to previous servo index. Cycle back to the top if on the 
    #last servo
    def CycleToPreviousServo(self):
        if(self.m_currentServoIndex != 0):
            self.m_currentServoIndex -= 1
        else:
            #Move back to top of list
            self.m_currentServoIndex = len(self.m_servoList) - 1

    #Move the current servo angle higher
    def MoveUp(self):
        currentServo = self.m_servoList[self.m_currentServoIndex]
        currentServo.MoveUp()
            
    #Move the current servo angle lower
    def MoveBack(self):
        currentServo = self.m_servoList[self.m_currentServoIndex]
        currentServo.MoveBack()

    def ReadyGrabPosition(self):
        #Grab Ready Position
        BasePanGrabAngle = 90
        ShoulderGrabAngle = 90
        ElbowGrabAngle = 20
        WristPanGrabAngle = 70
        WristGrabAngle = 5
        GripperGrabAngle = self.GripperMaxAngle
        GrabPositionArray = [BasePanGrabAngle, ShoulderGrabAngle, ElbowGrabAngle,
                         WristPanGrabAngle, WristGrabAngle, GripperGrabAngle]
        
        curAngleIndex = 0 #Todo, don't use index like this
        for curServo in self.m_servoList:
            servoAngle = GrabPositionArray[curAngleIndex]
            curServo.SetAngle(servoAngle)
            curAngleIndex += 1            

    def MagnetGrabPosition(self):
        #Magnet Pickup Positions
        BasePanMagnetGrabAngle = 90
        ShoulderMagnetGrabAngle = 160
        ElbowMagnetGrabAngle = 20
        WristPanMagnetGrabAngle = 90
        WristMagnetGrabAngle = 180
        gripperMagnetGrabAngle = self.GripperMaxAngle
        MagnetGrabPositionArray = [BasePanMagnetGrabAngle, ShoulderMagnetGrabAngle,
                               ElbowMagnetGrabAngle, WristPanMagnetGrabAngle, WristMagnetGrabAngle,
                               gripperMagnetGrabAngle]
    
        curAngleIndex = len(self.m_servoList)-1 #Todo, don't use index like this
        for curServo in  reversed(self.m_servoList):
            servoAngle = MagnetGrabPositionArray[curAngleIndex]
            curServo.SetAngle(servoAngle)
            curAngleIndex -= 1
            time.sleep(.5) #don't move it all at once for now
            
    #Move all the servos back to their home position
    def HomePosition(self):
        for curServoIndex in range(len(self.m_servoList)):
            servo = self.m_servoList[curServoIndex]
            #don't move the gripper index
            if (curServoIndex != 5):
                servo.MoveToHome()
            #time.sleep(.5) #don't move it all at once for now

    #Move all servos to their drop position
    def DropPosition(self):
        #Bucket Drop Positions
        BasePanDropAngle = 90
        ShoulderDropAngle = 90
        ElbowDropAngle = 170
        WristPanDropAngle = 90
        WristDropAngle = 170
        GripperGrabAngle = self.GripperMaxAngle
        DropPositionArray = [BasePanDropAngle, ShoulderDropAngle, ElbowDropAngle, WristPanDropAngle,
                         WristDropAngle, GripperGrabAngle]
    
        curAngleIndex = 0
        for curServo in self.m_servoList:
            servoAngle = DropPositionArray[curAngleIndex]
            curServo.SetAngle(servoAngle)
            curAngleIndex += 1
            time.sleep(.5) #don't move it all at once for now

    def CurrentlyOnPanServo(self):
        return (self.m_currentServoIndex == self.BasePanIndex or 
                	self.m_currentServoIndex == self.WristPanIndex)

    def CurrentlyOnTiltServo(self):
        return (self.m_currentServoIndex == self.ShoulderIndex or
		self.m_currentServoIndex == self.ElbowIndex or
                                self.m_currentServoIndex == self.WristIndex)

    def CurrentlyOnGripServo(self):
        return (self.m_currentServoIndex == self.GripperIndex)

    def GetCurrentServoIndex(self):
        return self.m_currentServoIndex

    def GetCurrentServoAngle(self):
        curServo = self.m_servoList[self.m_currentServoIndex]
        return curServo.GetAngle()

    def GetCurrentServo(self):
        return self.m_servoList[self.m_currentServoIndex]

    def SetCurrentServoAngle(self, angle):
        curServo = self.m_servoList[self.m_currentServoIndex]
        curServo.SetAngle(angle)

    
