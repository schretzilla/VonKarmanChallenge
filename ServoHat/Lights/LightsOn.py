#! /usr/bin/python
import Adafruit_PCA9685

#PWM constants
PWM = Adafruit_PCA9685.PCA9685(address=0x60)
PWM.set_pwm_freq(60)

#LED Pins
RedPin = 13
GreenPin = 14
BluePin = 15

MaxPower = 4095

#Turn on all leds at full power
def LightupLeds():
        PWM.set_pwm(RedPin, 0, MaxPower)
        PWM.set_pwm(GreenPin, 0, MaxPower)
        PWM.set_pwm(BluePin, 0, MaxPower)    
    
if __name__ == '__main__':
    LightupLeds()
