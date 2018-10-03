#! /usr/bin/python
import Adafruit_PCA9685
 
class Servo():
    #Encapsulate all servo data
    ServoMin = 150 #TODO: Use these and find them for each servo
    ServoMax = 600

    @staticmethod
    def initGlobals():
        global PWM
        #PWM constants
        PWM = Adafruit_PCA9685.PCA9685(address=0x60)
        PWM.set_pwm_freq(60)

    #Initializer: Sets the home angle to the provided current angle
    # then moves the servo to this specified angle
    def __init__(self, armIndex, currentAngle):
        self.initGlobals()
        self.m_armIndex = armIndex
        #The servo movement step size in degrees
        self.MovementStepSize = 5 
        self.m_homeAngle = currentAngle
        self.m_currentAngle = self.SetServoAngle(currentAngle)

    #Increase the servo's angle by the movement step size
    def MoveUp(self):
        newAngle = self.m_currentAngle + self.MovementStepSize
        self.m_currentAngle = self.SetServoAngle(newAngle)

    #Decrease the servo's angle by the movement step size
    def MoveBack(self):
        newAngle = self.m_currentAngle - self.MovementStepSize
        self.m_currentAngle = self.SetServoAngle(newAngle)

    #Increase the servo's angle by the movement step size
    def MoveToHome(self):
        self.m_currentAngle = self.SetServoAngle(self.m_homeAngle)

    #Method for updating the servo angle
    def SetServoAngle(self, angle):
        pulse = int(angle * 500.0/180.0 + 150.0)
        PWM.set_pwm(self.m_armIndex, 0, pulse)
        return angle

    def GetCurrentAngle(self):
        return self.m_currentAngle