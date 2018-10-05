#! /usr/bin/python

"""
Arm Joystick controller
Handles recieving and interpreting commands from the joystick
"""
from Arm import Arm
import rospy
from sensor_msgs.msg import Joy
import time


#The value provided when a button is activated
ActivatedButton = 1
ActivatedTrigger= -1

#The deadzone value for which joystick commands are ignored
JoyDeadzone = .05

#XBox Button Array Indicies
ButtonA = 0
ButtonB = 1
ButtonX = 2
ButtonY = 3

#XBox Controller Axes Array Indicies
HorizontalRightJoy = 2
VerticalRightJoy = 4
RightTrigger = 5
LeftTrigger = 2

def Callback(data):
    #Only accept arguments if the left trigger is active
    if data.axes[LeftTrigger] == ActivatedTrigger:
        
        if data.buttons[ButtonA] == ActivatedButton:
            #Cycle Forward
            m_arm.CycleToNextServo()
            rospy.loginfo("Cycle to next servo")
            rospy.loginfo("Current Servo is " + str(m_arm.GetCurrentServoIndex()))

        elif data.buttons[ButtonB] == ActivatedButton:
            #Cycle Back one servo
            m_arm.CycleToPreviousServo()
            rospy.loginfo("Cycle to previous servo")
            rospy.loginfo("Current Servo is " + str(m_arm.GetCurrentServoIndex()))
            
        elif (
                  data.buttons[ButtonY] == ActivatedButton 
                  and data.axes[RightTrigger] == ActivatedTrigger
                ):
            #Move to Home Position
            m_arm.MoveToHomePosition()
            rospy.loginfo("Move arm to home position")

        elif (m_arm.CurrentlyOnPanServo()):
            #Pan Movement
            rospy.loginfo("Move horizontally")
            rightJoyHorizontalInput = data.axes[HorizontalRightJoy]
            HandleJoystickInput(rightJoyHorizontalInput)

        elif (m_arm.CurrentlyOnTiltServo()):
            #Tilt Movement
            rospy.loginfo("Move vertically")
            rightJoyVerticalInput = data.axes[VerticalRightJoy]
            HandleJoystickInput(rightJoyVerticalInput)

        elif (m_arm.CurrentlyOnGripServo()):
            #Handle gripper movement
            rospy.loginfo("Handle gripper movement")
            rightJoyHorizontalInput = data.axes[HorizontalRightJoy]
            HandleJoystickInput(rightJoyVerticalInput)

"""
Handle Joystick input
Move the current servo up or back baised on joystick position.
"""
def HandleJoystickInput(joystickInput):
    if(joystickInput > JoyDeadzone):
        m_arm.MoveUp()
    elif(joystickInput < -JoyDeadzone):
        m_arm.MoveBack()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('joy', Joy, Callback)
    rospy.spin()

    
if __name__ == '__main__':
    m_arm = Arm()
    listener()
