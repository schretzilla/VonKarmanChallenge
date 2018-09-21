# VonKarmanChallenge

## Dependencies
### On the Master Computer
- ROS Kinetic
- [ROS Joy](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) - for the X-Box controller interaction

### Python
- zbar
- pyzbar
- numpy
- imutils
- opencv-contrib-python

### On the Raspberry Pi 3 B:
- [ROS Kinetic Image](https://downloads.ubiquityrobotics.com/pi.html)
- Arduino IDE Installed
- [ROS Serial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)


## Driving the Rover
For reference it would be best to review [Running ROS on Multiple Machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
### On the Master Computer
1. Start roscore
2. Set your ROS_MASTER_URI=http://{master computer's IP}:11311
3. run the joy_node [Described Here](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

### SSH to the PI
1. Set your ROS_MASTER_URI=http://{master computer's IP}:11311
2. Run the rosserial client for the nano controlling the Tank Drive
    - $ rosrun rosserial_python serial_node.py /dev/


## Streaming Video from the Pi Camera with Netcat
### On the Master Computer 
1. nc -l 2222 | mplayer -fps 200 -demuxer h264es -

### On the Pi
1. /opt/vc/bin/raspivid -t 0 -w 300 -h 300 -hf -vf -fps 20 -o - | nc 192.168.1.53 2222
