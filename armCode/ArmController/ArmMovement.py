#! /usr/bin/python

from Arm import Arm
import rospy
from sensor_msgs.msg import Joy
import time


#The value provided when a button is activated
ActivatedButton = 1

#The deadzone value for which joystick commands are ignored
JoyDeadzone = .05

#XBox Button Array Indicies
ButtonA = 0
ButtonB = 1
ButtonX = 2
ButtonY = 3

#XBox Controller Axis Array Indicies
HorizontalRightJoy = 2
VerticalRightJoy = 3
RightTriger = 4
LeftTrigger = 5

def callback(data):
    #Only accept arguments if the left trigger is active
    if data.buttons[LeftTrigger] == ActivatedButton:
        if data.buttons[ButtonA] == ActivatedButton:
            #Cycle Forward
            self.m_arm.CycleToNextServo()
            rospy.loginfo(self.m_arm.CurrentServoIndex())

    elif data.buttons[ButtonB] == ActivatedButton:
        #Cycle Back one servo
        self.m_arm.CycleToPreviousServo()
        rospy.loginfo(self.m_arm.CurrentServoIndex())
    elif (
              data.Buttons[ButtonY] == ActivatedButton 
              and data.Axis[RightTrigger] == ActivatedButton
            ):
        self.m_arm.MoveToHomePosition()

    elif (self.m_arm.CurrentlyOnPanServo()):
        #Pan Movement
        rightJoyHorizontalInput = data.Axis[HorizontalRightJoy]
        HandleJoystickInput(rightJoyHorizontalInput)

    elif (self.m_arm.CurrentlyOnTiltServo()):
        #Tilt Movement
        rightJoyVerticalInput = data.Axis[VerticalRightJoy]
        HandleJoystickInput(rightJoyVerticalInput)

    elif (self.m_arm.CurrentlOnGripServo()):
        rightJoyHorizontalInput = data.Axis[HorizontalRightJoy]
        HandleJoystickInput(rightJoyVerticalInput)

#rospy.loginfo(data.buttons[0])

def HandleJoystickInput(joystickInput):
    if(rightJoyVerticalInput > JoyDeadzone):
        self.m_arm.MoveUp()
    elif(rightJoyVerticalInput < -JoyDeadzone):
        self.m_arm.MoveBack()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)
    rospy.spin()

def OutputServoIndex():
    print("Current Servo Index: " + str(m_arm.CurrentServoIndex()))

#Test that Cycle Up works as expected
#Cycle up circles back to zero after the end of it's range
def TestCycleUp():
    print("Test Cycle Up")
    failed = False

    #Cycle around all servos twice
    for cycleNum in range(0, 2):
        #Through each servo
        for x in range(0, 6):
            OutputServoIndex()
            if(x != m_arm.CurrentServoIndex()):
                failed = True
            m_arm.CycleToNextServo()
            
    if(failed):
        print("Test cycle up Failed")
    else:
        print("Test cycle up Passed")

#Test that Cycle Down works as moves down one servo at a time
#Cycle up circles back to zero after the end of it's range
def TestCycleDown():
    print("Test Cycle Down")
    failed = False
    numOfServos = 6
    for cycleNum in range(0, 2):
        #Iterate backwords though all servos
        expectedServoIndex = numOfServos
        for x in range(0, numOfServos):
            m_arm.CycleToPreviousServo()
            expectedServoIndex -= 1
            OutputServoIndex()
            if( expectedServoIndex != m_arm.CurrentServoIndex()):
                failed = True
                print(str(x) + " is not equal to index " + str(expectedServoIndex))
        
    if(failed):
        print("Test Cycle Down Failed")
    else:
        print("Test cycle Down Passed")
    
def TestSetAngle():
    servoAngleStepSize = m_arm.Get

def TestMethods():
    OutputServoIndex()
    TestCycleUp()
    TestCycleDown()
    
    
if __name__ == '__main__':
    m_arm = Arm()
    TestMethods()
    #userInput = raw_input("Direction up (u) or down (d) or q to quit:")
    #while(userInput != "q"):
      #  if(userInput == "d"):
        #    userInput = raw_input("Direction up (u) or down (d) or q to quit:")
          #  m_arm.MoveBack()
            

    #SetServoAngle(WristIndex, 50)
    #listener()
    #StartUpWave()
