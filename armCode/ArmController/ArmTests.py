#! /usr/bin/python

#Test Methods for the 6 Count Servo Arm

from Arm import Arm
from Servo import Servo

def OutputTestStatus(passed, testName):
    passedMsg = "Passed"
    if not passed:
        passedMsg = "Failed"
    print(testName + " " + passedMsg)

def OutputServoIndex():
    print("Current Servo Index: " + str(m_arm.GetCurrentServoIndex()))

def OutputServoAngle():
    print("Current Servo Angle " + str(m_arm.GetCurrentServoAngle()))

#Test that Cycle Up works as expected
#Cycle up circles back to zero after the end of it's range
def TestCycleUp():
    print("Test Cycle Up")
    passed = True

    #Cycle around all servos twice
    for cycleNum in range(0, 2):
        #Through each servo
        for x in range(0, 6):
            OutputServoIndex()
            if(x != m_arm.GetCurrentServoIndex()):
                passed = false
            m_arm.CycleToNextServo()

    OutputTestStatus(passed, "Cycle Up Test")  
    #if(failed):
     #   print("Test cycle up Failed")
    #else:
     #   print("Test cycle up Passed")

#Test that Cycle Down works as moves down one servo at a time
#Cycle up circles back to zero after the end of it's range
def TestCycleDown():
    print("Test Cycle Down")
    passed = True
    numOfServos = 6
    for cycleNum in range(0, 2):
        #Iterate backwords though all servos
        expectedServoIndex = numOfServos
        for x in range(0, numOfServos):
            m_arm.CycleToPreviousServo()
            expectedServoIndex -= 1
            OutputServoIndex()
            if( expectedServoIndex != m_arm.GetCurrentServoIndex()):
                passed = False
                print(str(x) + " is not equal to index " + str(expectedServoIndex))

    OutputTestStatus(passed, "Cycle Down Test")  

def TestMoveServoBack():
    #Setup
    print("Test Move Servo Back")
    curServo = m_arm.GetCurrentServo()
    startingAngle = m_arm.GetCurrentServoAngle()
    servoStepSize = curServo.MovementStepSize
    desiredAngle = startingAngle - curServo.MovementStepSize
    movementAllowed = curServo.IsAngleAllowed(desiredAngle)
    
    #Test move servo back
    OutputServoAngle()
    m_arm.MoveBack()
    OutputServoAngle()

    passed = False
    if(movementAllowed):
        passed = m_arm.GetCurrentServoAngle() == startingAngle - servoStepSize
    else:
        passed = m_arm.GetCurrentServoAngle() == startingAngle

    OutputTestStatus(passed, "Move Angle Back Test")

def TestMoveServoForward():
    #Setup
    print("Test Move Servo Forward")
    curServo = m_arm.GetCurrentServo()
    startingAngle = m_arm.GetCurrentServoAngle()
    servoStepSize = curServo.MovementStepSize
    desiredAngle = startingAngle + curServo.MovementStepSize
    movementAllowed = curServo.IsAngleAllowed(desiredAngle)
    
    #Test move servo forward
    OutputServoAngle()
    m_arm.MoveUp()
    OutputServoAngle()

    passed = False
    if(movementAllowed):
        passed = m_arm.GetCurrentServoAngle() == startingAngle + servoStepSize
    else:
        passed = m_arm.GetCurrentServoAngle() == startingAngle
        
    OutputTestStatus(passed, "Move Angle Forward Test")

def TestMoveServo():
    #Setup
    curServo = m_arm.GetCurrentServo()
    startingAngle = 80
    curServo.SetServoAngle(startingAngle)

    TestMoveServoForward()
    TestMoveServoBack()

    bigAngle = curServo.MaxAllowedAngle
    curServo.SetServoAngle(bigAngle)
    TestMoveServoForward()
    TestMoveServoBack()

    smallAngle = curServo.MinAllowedAngle
    curServo.SetServoAngle(smallAngle)
    TestMoveServoBack()
    TestMoveServoForward()
    
def TestMethods():
    OutputServoIndex()
    TestCycleUp()
    TestCycleDown()
    TestMoveServo()
    
if __name__ == '__main__':
    m_arm = Arm()
    TestMethods()

