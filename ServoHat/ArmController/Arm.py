#! /usr/bin/python

from Servo import Servo
import time

class Arm():
    #Servo Indicies
    BasePanIndex = 0
    BaseHomeAngle = 30
    BaseMinAngle = 0
    BaseMaxAngle = 180

    ShoulderIndex = 1
    ShoulderHomeAngle = 30
    ShoulderMinAngle = 0
    ShoulderMaxAngle = 180

    ElbowIndex = 2
    ElbowHomeAngle = 30
    ElbowMinAngle = 0
    ElbowMaxAngle = 180
    
    WristPanIndex = 3
    WristPanHomeAngle = 30
    WristPanMinAngle = 0
    WristPanMaxAngle = 180

    WristIndex = 4
    WristHomeAngle = 30
    WristMinAngle = 10
    WristMaxAngle = 120
    
    GripperIndex = 5
    GripperHomeAngle = 30
    GripperMinAngle = 16
    GripperMaxAngle = 103

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
        
    #Move all the servos back to their home position
    def MoveToHomePosition(self):
        for servo in self.m_servoList:
            servo.MoveToHome()
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
        return curServo.GetCurrentAngle()

    def GetCurrentServo(self):
        return self.m_servoList[self.m_currentServoIndex]

    def SetCurrentServoAngle(self, angle):
        curServo = self.m_servoList[self.m_currentServoIndex]
        curServo.SetServoAngle(angle)

    
