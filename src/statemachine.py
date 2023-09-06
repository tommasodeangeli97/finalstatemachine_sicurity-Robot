#!/usr/bin/env python

import rospy
import roslib
import rospkg
import smach
import smach_ros
import time
import math
import random
from std_msgs.msg import Bool, String
from armor_api.armor_client import ArmorClient

"""
.. module::state_machine
   :platform: Unix
   :synopis: the state machine to controll the behavior of the robot
   
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS state machine implemented with smach library with the purpose of controlling the behaviour of the robot; the robot must stays mostly in corridors and visites the rooms as they became 'urgent'; when the battery level is low the robot goes back to the initial room, the 'E' room, and charge itself

Publishes to:
	- /room(bool) it indicates that the robot is in the charging room: True value -> the robot is in the room; False value -> the robot is not in the right room
	
Subribes to:
	- /load(bool) it indicates if the map is ready: True value -> the ontology is ready; False value -> the ontology is not ready yet
	- /batttrigger(bool) it indicates if the battery is full or low: True value -> the battery is low; False value -> the battery is full
	
"""

client = ArmorClient("example", "ontoRef")
"""
opens the client for all the functions in the code
"""
wim = ''
"""
global variable that indicates the actual position of the robot
"""
ready = False
"""
global variable for the value of the /load topic
"""
battery = False
"""
global variable for the value of the /batttrigger topic
"""

def callback(data):
	"""
	callback function that changes the 'ready' variable
	"""
	global ready
	ready = data.data
	
def bat_callback(data):
	"""
	callback function that changes the 'battery' variable
	"""
	global battery
	battery = data.data
	
def view_time(time_list):
	"""
	function that clear the queried time

	Args:
		-time_list the list of queried time stamp
	Returns:
		the time_stamp usable value
	"""
	timestamp = ''
	for i in time_list:
		for element in range(1, 11):
			timestamp=timestamp+i[element]
	return timestamp
	
def visited(n_r):
	"""
	function that update the object propriety 'isIn' for the robot and the two data propriety 'now' and 'visitedAt'; at the end replace the robot in the staring position

	Args:
		-r_n the new room where the robot is
	"""
	client.call('REASON','','',[''])
	act_t = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
	old_t = view_time(act_t.queried_objects)
	now_t = str(math.floor(time.time()))
	client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', now_t, old_t])
	
	client.call('REASON','','',[''])
	
	room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', n_r[0]])
	old_room = view_time(room_time.queried_objects)
	
	client.call('REPLACE','DATAPROP','IND',['visitedAt',n_r[0], 'Long', now_t, old_room])
	client.call('REASON','','',[''])
	print(" visited ")
	return

def check_urgents():
	"""
	function that queries if there are 'urgent' room and checkes if the list is empty
	
	Returns:
		the list of all the 'urgent' rooms in that moment
	"""
	
	client.call('REASON','','',[''])
	calling = client.call('QUERY','IND','CLASS',['URGENT'])
	urg_list = []
	for i in calling.queried_objects:
		if "R1" in i:
			print(" urgent R1 ")
			urg_list.append('R1')
		elif "R2" in i:
			print(" urgent R2 ")
			urg_list.append('R2')
		elif "R3" in i:
			print(" urgent R3 ")
			urg_list.append('R3')
		elif "R4" in i:
			print(" urgent R4 ")
			urg_list.append('R4')
		
	if urg_list == []:
		print(" nothing urgent for the moment ", urg_list)
	else:
		print(" we have some urgent rooms ", urg_list)
	return urg_list
	
def check_near(pos):
	"""
	funtion that queries all the rooms that are near to the one where the robot is
	
	Args:
		-pos: the actual position of the robot
		
	Returns:
		the list of the rooms that share the same door with pos
	"""
	
	client.call('REASON','','',[''])
	calling = client.call('QUERY','OBJECTPROP','IND',['connectedTo', pos])
	near_list = []
	for i in calling.queried_objects:
		if "R1" in i:
			near_list.append('R1')
		elif "R2" in i:
			near_list.append('R2')
		elif "R3" in i:
			near_list.append('R3')
		elif "R4" in i:
			near_list.append('R4')
		elif "C1" in i:
			near_list.append('C1')
		elif "C2" in i:
			near_list.append('C2')
		elif "C3" in i:
			near_list.append('C3')
		elif "E" in i:
			near_list.append('E')
	return near_list
	
def move(cr, nr):
	"""
	function that changes the position of the robot
	
	Args:
		-cr: the room that the robot leaves
		-nr: the new room where the robot goes to
	"""
	
	print(" i'm moving from ", cr)
	print(" i'm moving to ", nr)
	client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', nr[0], cr])
	
	client.call('REASON','','',[''])
	
	rospy.sleep(1)
	
	return

class WaitMap(smach.State):
	"""
	Class that defines the 'RECIEVING_MAP' state, the robot just waits untill the variable 'ready' becames 'True' in that case the map is loaded and the robot leaves the 'E' room to go to the 'C1' corridor; it changes the state
	
	Args:
		-smach.State: base state interface
		
	Returns:
		-map_not_redy: transition condition that maintain the 'RECIEVING_MAP' status active
		-map_redy: transistion condition that change status from 'RECIEVING MAP' to 'PATROLLING_CORRIDOR'
	"""
	#initial state, the waiting for the map
	def __init__(self):
		smach.State.__init__(self, outcomes=['map_not_ready','map_ready'])
		
	def execute(self, userdata):
		global ready
		global wim
		rospy.sleep(2)
		rospy.Subscriber("load", Bool, callback)
		
		if ready == False:
			print("I'm still waitnig")
			return 'map_not_ready'
			
		else:
			print(" uploading the map ") 
			client.call('LOAD','FILE','',['/root/ros_ws/src/finite-state-machine---security-robot/topological_map/new_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
			print ("I'm leaving the E room")
			client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'E'])
			client.call('REASON','','',[''])
			act_t = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
			old_t = view_time(act_t.queried_objects)
			now_t = str(math.floor(rospy.get_time()))
			client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', now_t, old_t])
			client.call('REASON','','',[''])
			room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', 'C1'])
			old_room = view_time(room_time.queried_objects)
			client.call('REPLACE','DATAPROP','IND',['visitedAt','C1', 'Long', now_t, old_room])
			client.call('REASON','','',[''])
			wim = 'C1'
			return 'map_ready'			 
			

class PatrollingCorridor(smach.State):
	"""
	Class that defines the 'PATROLLING_CORRIDOR' state: it checks if the battery is low, in that case the status changes to 'BATTERY_LOW', if not, the robot checks its position, if there are urgent rooms and which are the room accessible; if there are urgent rooms and they are accessible the robot changes its position and the status changes to 'PATROLLING_ROOM'; if there are urgent rooms but the robot isn't near to them it checks which is the room that is near to the urgent rooms and change its position to this room, it remains in the 'PATROLLING_CORRIDOR' status; if there are no urgent rooms the robot prefers to stay in corridors, it changes corridors and remains in the 'PATROLLING_CORRIDOR' status
	
	Args:
		-smach.State: base state interface
		
	Returns:
		-still_corridor: transition condition that maintain the 'PATROLLING_CORRIDOR' status
		-in_room: transition condition that changes to 'PATROLLING_ROOM' status
		-allert_battery: transition condition that changes to 'BATTERY_LOW' status
	"""
	#state where the robot moves in the corridors and check if there are urgent room and in case goes to them
	def __init__(self):
		smach.State.__init__(self, outcomes=['still_corridor','in_room', 'allert_battery'])
		
	def execute(self, userdata):
		global battery
		global wim
		if battery == True:
				print(" the battert is low, i need charge ")
				return 'allert_battery'
		else:
			
			n_c = ''
			
			print(" wim = ",wim)
			wicg = check_near(wim)
			atu = check_urgents()
			c1 = 'C1'
			c2 = 'C2'
			c3 = 'C3'
			
			if atu == []:
				print(" let's stay in corridords so ")
				if wim == c1:
					corridors = ['C2','C3']
					n_c = random.choices(corridors, weights = [1,1])
					print(" next room is ", n_c[0])
					move(wim,n_c)
					visited(n_c)
					wim = n_c[0]
					print(" next corridor ")
					return 'still_corridor'
				elif wim == c2:
					corridors = ['C1','C3']
					n_c = random.choices(corridors, weights = [1,10])
					print(" next room is ", n_c)
					move(wim,n_c)
					visited(n_c)
					wim = n_c[0]
					print(" next corridor ")
					return 'still_corridor'
				elif wim == c3:
					corridors = ['C1','C2']
					n_c = random.choices(corridors, weights = [1,10])
					print(" next room is ", n_c)
					move(wim,n_c)
					visited(n_c)
					wim = n_c[0]
					print(" next corridor ")
					return 'still_corridor'
			else:
				n_r = []
				room_set = set(wicg) & set(atu)
				for i in room_set:
					if "R1" in i:
						n_r.append('R1')
					elif "R2" in i:
						n_r.append('R2')
					elif "R3" in i:
						n_r.append('R3')
					elif "R4" in i:
						n_r.append('R4')
					elif "C1" in i:
						n_r.append('C1')
					elif "C2" in i:
						n_r.append('C2')
					elif "C3" in i:
						n_r.append('C3') 
				if n_r:
					print(" the nearest urgent room is ", n_r)
					move(wim,n_r)
					visited(n_r)
					wim = n_r[0]
					return 'in_room'
				else:
					next_move = check_near(atu[0])
					room_set2 = set(wicg)&set(next_move)
					for j in room_set2:
						if "R1" in j:
							n_r.append('R1')
						elif "R2" in j:
							n_r.append('R2')
						elif "R3" in j:
							n_r.append('R3')
						elif "R4" in j:
							n_r.append('R4')
						elif "C1" in j:
							n_r.append('C1')
						elif "C2" in j:
							n_r.append('C2')
						elif "C3" in j:
							n_r.append('C3')
					print(" next room is ", n_r)
					move(wim,n_r)
					visited(n_r)
					wim = n_r[0]
					return 'still_corridor'
						
		
class PatrollingRoom(smach.State):
	"""
	Class that define the 'PATROLLING_ROOM' state: the robot chacks if the battery is low, in that case the status changes to 'BATTERY_LOW', if not, it waits a certain amount of time then checks in which room it is and which is the near room, it moves to that room and change the status to 'PATROLLING_CORRIDOR'
	
	Args:
		-smach.State: base state interface
		
	Returns:
		-back_to_corridor: transition condition that changes the status in 'PATROLLING_CORRIDOR'
		-allert_battery: transition condition that changes to 'BATTERY_LOW' status
	"""
	#state where the robot stays in the room for a bit 
	def __init__(self):
		smach.State.__init__(self, outcomes=['back_to_corridor', 'allert_battery'])
		
	def execute(self, userdata):
		global battery
		global wim
		if battery == True:
			print(" the battert is low, i need immediate charge ")
			return 'allert_battery'
		else:
			
			rospy.sleep(1)
			
			wicg = check_near(wim)
			print(" I have finished here ")
			move(wim,wicg)
			visited(wicg)
			wim = wicg[0]
			return 'back_to_corridor'
			

class BatteryLow(smach.State):
	"""
	Class that defines the 'BATTERY_LOW' status, it checks if the 'battery' variable is still 'True', in that case it checks the actual position and moves consequently remaining in the 'BATTERY_LOW' status: if the robot is in 'C1' it moves to the 'E' room and publish on the /room topic; if the robot is not in 'C1' it chooses the most rapid path to go to the 'E' room and changes one room at time; when the 'battery' becames 'False' the robot goes to the 'C1' room and change to 'PATROLLING_CORRIDOR' state
	
	Args:
		-smach.State: base state interface
	
	Returns:
		-back_to_work: transition condition that changes to 'PATROLLING_CORRIDOR'
		-allert_battery: transition condition that maintain the 'BATTERY_LOW' status
	"""
	#state triggered whenever the battery is low
	def __init__(self):
		smach.State.__init__(self, outcomes=['back_to_work', 'allert_battery'])
	
	def execute(self, userdata):
		global battery
		global wim
		pub=rospy.Publisher('room', Bool, queue_size=10)
		if battery == True:
			
			if wim == 'C1':
				home = ['E']
				move(wim,home)
				visited(home)
				wim = home[0]
				#pub=rospy.Publisher('room', Bool, queue_size=10)
				pub.publish(True)
			elif wim == 'C2' or wim == 'C3':
				next_move = ['C1']
				move(wim,next_move)
				visited(next_move)
				wim = next_move[0]
				print(" I'm going to charge myself ")
			elif wim == 'E':
				print(" charging ")
				pub.publish(True)
				rospy.sleep(10)
			else:
				corridors = ['C2','C3']
				wicg = check_near(wim)
				n_r = []
				room_set = set(wicg)&set(corridors)
				for i in room_set:
					if "R1" in i:
						n_r.append('R1')
					elif "R2" in i:
						n_r.append('R2')
					elif "R3" in i:
						n_r.append('R3')
					elif "R4" in i:
						n_r.append('R4')
					elif "C1" in i:
						n_r.append('C1')
					elif "C2" in i:
						n_r.append('C2')
					elif "C3" in i:
						n_r.append('C3')
				print(" going to charge ")
				move(wim,n_r)
				visited(n_r)
				wim = n_r[0]
			return 'allert_battery'
		else:
			print(" Let's start again ")
			home = 'E'
			next = ['C1']
			move(home,next)
			visited(next)
			wim = next[0]
			pub.publish(False)
			return 'back_to_work'
			
			
def main():
	"""
	the main function that initialises the node and the state machine, it declares all the conjunctions between the state, it declares the subscribers
	"""
	#the main function
	rospy.sleep(5)
	rospy.init_node('state_machine')
	 
	sm = smach.StateMachine(outcomes=['Interface'])
	
	with sm:
		# Add states to the container
		smach.StateMachine.add('RECIEVING_MAP', WaitMap(), 
			transitions={'map_not_ready':'RECIEVING_MAP', 'map_ready':'PATROLLING_CORRIDOR'})
		smach.StateMachine.add('PATROLLING_CORRIDOR', PatrollingCorridor(), 
			transitions={'still_corridor':'PATROLLING_CORRIDOR', 'in_room':'PATROLLING_ROOM', 'allert_battery':'BATTERY_LOW'})
		smach.StateMachine.add('PATROLLING_ROOM', PatrollingRoom(), 
			transitions={'back_to_corridor':'PATROLLING_CORRIDOR', 'allert_battery':'BATTERY_LOW'})
		smach.StateMachine.add('BATTERY_LOW', BatteryLow(), 
			transitions={'back_to_work':'PATROLLING_CORRIDOR', 'allert_battery':'BATTERY_LOW'})
			
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	#rospy.Subscriber("load", Bool, callback)
	rospy.Subscriber("batttrigger", Bool, bat_callback)
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
	
if __name__ == '__main__':
	"""
	the entry point for the module
	"""
	#rospy.init_node('state_machine')
	main()
		
	
