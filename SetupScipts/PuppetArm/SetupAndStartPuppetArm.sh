MY_IP=$1
if [ $# -eq 0 ]
then 
    echo "Environment Variables not setup!"
    echo "Provide the Masters IP address as arg 1"
else
    export ROS_IP=$MY_IP
    echo "ROS_IP=$ROS_IP"
    export ROS_MASTER_URI="http://$MY_IP:11311"
    export ROS_NAMESPACE="puppet"
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "PuppetArm setup"
    rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=250000
fi
