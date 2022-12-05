# [... original bashrc stuff ...]

export ROS_IP="$(hostname -I)" #'192.168.15.133'
export ROS_HOSTNAME="$(hostname -I)"
export ROS_ROBOT='10.42.0.1'
#export ROS_MASTER_URI="http://$ROS_IP:11311" # enable when sim
export ROS_MASTER_URI="http://$ROS_ROBOT:11311" # enable when physical

# VMWare settings:
# Network Adapter: set to NAT when running sim, set to Bridged when running on physical robot