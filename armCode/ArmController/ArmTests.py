#! /usr/bin/python

#Test Methods for the 6 Count Servo Arm

from Arm import Arm
from Servo import Servo

def OutputServoIndex():
    print("Current Servo Index: " + str(m_arm.GetCurrentServoIndex()))

def OutputServoAngle():
    print("Current Servo Angle " + str(m_arm.GetCurrentServoAngle()))

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
            if(x != m_arm.GetCurrentServoIndex()):
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
            if( expectedServoIndex != m_arm.GetCurrentServoIndex()):
                failed = True
                print(str(x) + " is not equal to index " + str(expectedServoIndex))
        
    if(failed):
        print("Test Cycle Down Failed")
    else:
        print("Test cycle Down Passed")
    
def TestSetAngle():
    curServo = m_arm.GetCurrentServo()
    servoStepSize = curServo.MovementStepSize
    startingAngle = m_arm.GetCurrentServoAngle()
    
    OutputServoAngle
    m_arm.MoveBack()
    OutputServoAngle

    passed = m_arm.GetCurrentServoAngle() == startingAngle - servoStepSize
    if(passed):
        print("Test Move Angle Back Passed")
    else:
        print("Test Move Angle Back Failed")
        
    
def TestMethods():
    OutputServoIndex()
    TestCycleUp()
    TestCycleDown()
    TestSetAngle()
    
if __name__ == '__main__':
    m_arm = Arm()
    TestMethods()

