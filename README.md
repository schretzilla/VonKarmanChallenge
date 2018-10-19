# VonKarmanChallenge

## Software Dependencies
### On the Master Computer
- ROS Kinetic
- [ROS Joy](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) - for the X-Box controller interaction

### Python
- zbar
- pyzbar
- numpy
- imutils
- opencv-contrib-python
- gtty
- pygame

### On the Raspberry Pi 3 B:
- [ROS Kinetic Image](https://downloads.ubiquityrobotics.com/pi.html)
- Arduino IDE Installed
- [ROS Serial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

## Hardware Dependencies
- An Xbox Controller for driving the rover
- The Model puppet 

## Driving the Rover
Once all dependencies are downloaded driving the rover is as easy as running the Driver script
$ SetupScripts/SingleCommandStartup/MarvOn.sh {Your Computer's IP address here}

### SSH to the PI
1. Set your ROS_MASTER_URI=http://{master computer's IP}:11311
2. Run the rosserial client for the nano controlling the Tank Drive
    - $ rosrun rosserial_python serial_node.py /dev/


## Streaming Video from the Pi Camera with Netcat
### On the Master Computer 
1. nc -l 2222 | mplayer -fps 200 -demuxer h264es -

### On the Pi
1. /opt/vc/bin/raspivid -t 0 -w 300 -h 300 -hf -vf -fps 20 -o - | nc 192.168.1.53 2222
