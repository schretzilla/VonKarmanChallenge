#! /usr/bin/python

from Servo import Servo
import time

class Arm():
	#Servo Indicies
	BasePanIndex = 0
	BaseHomeAngle = 30

	ShoulderIndex = 1
	ShoulderHomeAngle = 30

	ElbowIndex = 3
	ElbowHomeAngle = 30

	WristPanIndex =4
	WristPanHomeAngle = 30

	WristIndex = 5
	WristHomeAngle = 30

	GripperIndex = 6
	GripperHomeAngle = 30


	def __init__(self):
		baseServo = Servo(1, 2)
		shoulderServo = Servo(ShoulderIndex, ShoulderHomeAngle)
		elbowServo = Servo(ElbowIndex, ElbowHomeAngle)
		wristPanServo = Servo(WristPanIndex, WristPanHomeAngle)
		wristServo = Servo(WristIndex, WristHomeAngle)
		gripperServo = Servo(GripperIndex, GripperHomeAngle)
		self.m_servoList = [baseServo, shoulderServo, elbowServo,
		wristPanServo, wristServo, gripperServo]
		self.m_currentServoIndex = 0

	#Move to next servo index. Cycle back to zero if on the last servo
	def CycleToNextServo(self):
		if(self.m_currentServoIndex < len(self.m_servoList)):
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
			#move back to top of list
			self.m_currentServoIndex = len(self.m_servoList) - 1

	#Move the current servo angle higher
	def MoveUp(self):
		currentServo = self.m_servoList[m_currentServoIndex]
		self.m_currentServo.MoveUp

	#Move the current servo angle lower
	def MoveBack(self):
		currentServo = self.m_servoList[m_currentServoIndex]
		self.m_currentServo.MoveBack

	#Move all the servos back to their home position
	def MoveToHomePosition(self):
		for servo in self.m_servoList:
			servo.MoveToHome()
			time.sleep(.5) #don't move it all at once for now

	def CurrentlyMovingHorizontally(self):
		return (self.m_currentServoIndex == BasePanIndex or 
						self.m_currentServoIndex == WristPanIndex)

	def CurrentlyTiltServo(self):
		return (self.m_currentServoIndex == ShoulderIndex or
						self.m_currentServoIndex == ElbowIndex or
						self.m_currentServoIndex == WristIndex)

	def CurrentlyGripServo(self):
		return (self.m_currentServoIndex == GripperIndex)
