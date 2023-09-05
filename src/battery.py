#!/usr/bin/env python

import rospy
import roslib
import random
from std_msgs.msg import Bool, String

"""
.. module::battery_node
   :platform: Unix
   :synopis: a module for publishing the status of the battery
   
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS node that change the boolean variable 'battery' wich indicate the battery level of the robot; the node waits a random time after the robot leaves the charging room and publish the True value to indicate the discharging of the battery

Publishes to:
	- /batttrigger boolean value
	
Subribes to:
	- /room bulean value that indicates the presence of the robot in the charging room ('E' room)
	
"""

room = False
"""
variable room: boolean variable, True if the robot is actually in the 'E' room and can charge itself
"""

def pos_callback(data):
	"""
	callback function for updating the value of 'room'
	"""
		
	global room
	room = data.data
			
		
def main():
	"""
	main function in which the value of 'room' is checked and if it is True (the robot is in the right room) the robot charge for the indicated time and publishs that the battery is full -/batttrigger=False, if it is False the robot discharge for a random time and publishes the low battery value -/batttrigger=True
	"""
	global room
	rospy.init_node('battery_node', anonymous=True)
	pub = rospy.Publisher('batttrigger', Bool, queue_size=10)
	rospy.Subscriber('room', Bool, pos_callback)
	while not rospy.is_shutdown():
		
		if room == True:
			rospy.sleep(3)
			batt = False
			print(" recharged ")
			pub.publish(batt)
		else:
			autonomy = random.uniform(40,50)
			rospy.sleep(autonomy)
			batt = True
			print(" need recharge ")
			pub.publish(batt)
	
		
		

if __name__ == '__main__':
	"""
	The entry point of the node
	"""
	try:
		main()
	except rospy.ROSInterruptException:
		pass	
