#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty as msg_empty
from std_srvs.srv import Empty as srv_empty

def main():
	rospy.init_node("reset_odom", anonymous=False)

	pub = rospy.Publisher('mobile_base/commands/reset_odometry', msg_empty, 
			queue_size=10)

	e = msg_empty()

	r = rospy.Rate(1)
	r.sleep()
	
	pub.publish(e)

	# maybe do some 'wait for service' here
	rospy.wait_for_service('/gazebo/reset_world')
	reset_world = rospy.ServiceProxy('/gazebo/reset_world', srv_empty)

	reset_world()



if __name__ == '__main__':
	main()

