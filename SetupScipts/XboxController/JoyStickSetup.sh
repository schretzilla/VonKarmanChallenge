MY_IP=$1
if [ $# -eq 0 ]
then 
    echo "Environment Variables not setup!"
    echo "Provide the Masters IP address as arg 1"
else
    export ROS_IP=$MY_IP
    echo "ROS_IP=$ROS_IP"
    export ROS_MASTER_URI="http://$MY_IP:11311"
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    rosparam set joy_node/dev "/dev/input/js1"
    echo "joy node attached to /dev/input/js1"
fi