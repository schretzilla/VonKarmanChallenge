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



  #data.buttons[#]
  '''
  rospy.loginfo(data.buttons[0])
  rospy.loginfo(data.buttons[1])
  rospy.loginfo(data.buttons[2])
  rospy.loginfo(data.buttons[3])
  # set angles up
  if data.buttons[0]:
    rospy.loginfo("Position 0")
    SetServoAngle(0,70)
    SetServoAngle(1,90)
  elif data.buttons[1]:
    rospy.loginfo("Position 1")
    SetServoAngle(0,70)
    SetServoAngle(1,135)
  '''

def HandleJoystickInput(joystickInput):
  if(rightJoyVerticalInput > JoyDeadzone):
    self.m_arm.MoveUp()
  elif(rightJoyVerticalInput < -JoyDeadzone):
    self.m_arm.MoveBack()

def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber('joy', Joy, callback)
  rospy.spin()

if __name__ == '__main__':
  m_arm = Arm()
  userInput = raw_input("Direction up (u) or down (d) or q to quit:")
  while(userInput != "q"):
    if(userInput == "d"):
      userInput = raw_input("Direction up (u) or down (d) or q to quit:")
      m_arm.MoveBack()

    #SetServoAngle(WristIndex, 50)
    #listener()
    #StartUpWave()
