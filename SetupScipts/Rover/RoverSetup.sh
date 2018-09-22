MASTER_IP=$1
if [ $# -eq 0 ]
then
    echo "MASTER_IP not set"
    echo "Provide the masters IP address as arg 1"
else
    export ROS_MASTER_URI="http://$MASTER_IP:11311"
    echo $ROS_MASTER_URI
fi