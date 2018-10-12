#! /usr/bin/python
import Adafruit_PCA9685
 
class Servo():
    #Encapsulate all servo data

    @staticmethod
    def initGlobals():
        global PWM
        #PWM constants
        PWM = Adafruit_PCA9685.PCA9685(address=0x60)
        PWM.set_pwm_freq(60)

    #Initializer: Sets the home angle to the provided current angle
    # then moves the servo to this specified angle
    def __init__(self, armIndex, currentAngle, minAllowedAngle=None, maxAllowedAngle=None):
        #Set default bounds if not inputted
        if minAllowedAngle == None:
            minAllowedAngle = 0
        if maxAllowedAngle == None:
            maxAllowedAngle = 180
            
        self.initGlobals()
        #Todo: pass these in as params
        self.MinAllowedAngle = minAllowedAngle
        self.MaxAllowedAngle = maxAllowedAngle
        self.m_armIndex = armIndex
        #The servo movement step size in degrees
        self.MovementStepSize = 1
        self.m_homeAngle = currentAngle
        self.m_currentAngle = currentAngle
        self.SetAngle(currentAngle)


    #Increase the servo's angle by the movement step size
    #Only moves if within the servos angle bounds
    def MoveUp(self):
        newAngle = self.m_currentAngle + self.MovementStepSize
        self.m_currentAngle = self.SetAngle(newAngle)

    #Decrease the servo's angle by the movement step size
    #Only moves if within the servos angle bounds
    def MoveBack(self):
        newAngle = self.m_currentAngle - self.MovementStepSize
        self.m_currentAngle = self.SetAngle(newAngle)

    #Increase the servo's angle by the movement step size
    def MoveToHome(self):
        self.m_currentAngle = self.SetAngle(self.m_homeAngle)

    #Method for updating the servo angle
    def SetAngle(self, angle):
        movementAllowed = self.IsAngleAllowed(angle)
        #only move if it is within bounds
        if(movementAllowed):    
            pulse = int(angle * 500.0/180.0 + 150.0)
            PWM.set_pwm(self.m_armIndex, 0, pulse)
            self.m_currentAngle = angle

        return self.m_currentAngle

    def IsAngleAllowed(self, angle):
        return (self.MinAllowedAngle <= angle <= self.MaxAllowedAngle)
    
    def GetCurrentAngle(self):
        return self.m_currentAngle
