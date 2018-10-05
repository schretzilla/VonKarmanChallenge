#! /usr/bin/python

from Servo import Servo
import time

class Arm():
    #Servo Indicies
    BasePanIndex = 0
    BaseHomeAngle = 100

    ShoulderIndex = 1
    ShoulderHomeAngle = 100

    ElbowIndex = 2
    ElbowHomeAngle = 100

    WristPanIndex = 3
    WristPanHomeAngle = 100

    WristIndex = 4
    WristHomeAngle = 100

    GripperIndex = 5
    GripperHomeAngle = 100


    def __init__(self):
        baseServo = Servo(self.BasePanIndex, self.BaseHomeAngle)
        shoulderServo = Servo(self.ShoulderIndex, self.ShoulderHomeAngle)
        elbowServo = Servo(self.ElbowIndex, self.ElbowHomeAngle)
        wristPanServo = Servo(self.WristPanIndex, self.WristPanHomeAngle)
        wristServo = Servo(self.WristIndex, self.WristHomeAngle)
        gripperServo = Servo(self.GripperIndex, self.GripperHomeAngle)
        self.m_servoList = [baseServo, shoulderServo, elbowServo,
        wristPanServo, wristServo, gripperServo]
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
    
