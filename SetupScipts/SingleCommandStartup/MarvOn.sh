#!/bin/bash

export MASTER_IP=$1
exit_script() {
  echo "cleaning up"
  trap - SIGINT SIGTERM
  kill %1
  kill %2
}
if [ $# -eq 0 ]
then
    echo "MASTER_IP not set"
    echo "Provide the masters IP address as arg 1"
else
   trap exit_script SIGINT SIGTERM

   command sh -c 'export ROS_MASTER_URI="http://$MASTER_IP:11311"
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    rosparam set joy_node/dev "/dev/input/js1"
    echo "joy node attached to /dev/input/js1"
    echo "Starting Joy Node"
    rosrun joy joy_node' &
    
   echo "Starting SSH"
   command sh -c "ssh ubuntu@192.168.0.10 \"/home/ubuntu/Development/VonKarmanChallenge/SetupScipts/Rover/SetupAndStartRover.sh $MASTER_IP\"" &
   echo "Finished setup"
fi


echo "Ctr + C to exit"

sleep infinity
