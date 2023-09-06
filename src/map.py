#!/usr/bin/env python

import rospy
import time
import math
from std_msgs.msg import Bool
from armor_api.armor_client import ArmorClient

"""
.. module::map_node
   :platform: Unix
   :synopis: a module to creat the ontology where the robot moves
   
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS node that load an ontology and creates another with different instances, data propriety and object propriety using the armor_api client service; after that it send a boolean value to indicate the right functioning of the procedure 

Publishes to:
    - /load(bool) indicates that the map is updated and ready to be used
	
"""

client = ArmorClient("example", "ontoRef")
"""
client variable that opens the armor client
"""

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

def visited(room_list):
    """
    function that update the object propriety 'isIn' for the robot and the two data propriety 'now' and 'visitedAt'; at the end replace the robot in the staring position
    
    Args:
        -room_list the list of the rooms in the ontology without the 'E' room
    """
    c_r = 'E'
    for i in range(0,len(room_list)):
        client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', room_list[i], c_r])
        client.call('REASON','','',[''])
        
        act_t = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        old_t = view_time(act_t.queried_objects)
        now_t = str(math.floor(time.time()))
        client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', now_t, old_t])
        
        client.call('REASON','','',[''])
        
        client.call('ADD','DATAPROP','IND',['visitedAt',room_list[i], 'Long', now_t])
        client.call('REASON','','',[''])
        
        c_r = room_list[i]
        #i = i+1
        rospy.sleep(1)
        
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'E', c_r])
    client.call('REASON','','',[''])
    
    act_t = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_t = view_time(act_t.queried_objects)
    now_t = str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', now_t, old_t])
    client.call('REASON','','',[''])
    
    client.call('ADD','DATAPROP','IND',['visitedAt','E', 'Long', now_t])
    client.call('REASON','','',[''])

def loading():
    """
    main function that initializes the node and the publisher; it loads the ontology and creates the instances and the object proprieties; it disjoints the varius rooms; it calls the visited() function and then it creates and saves in a new ontology; it publishes the True value on the -/load topic
    """
    rospy.init_node('map_node', anonymous=True)
    pub = rospy.Publisher('load', Bool, queue_size=10)
    pub.publish(False)
    
    print (" Loading the map ")
    
    #client = ArmorClient("tommaso", "ontology")
    
    #loading the map statically
    
    client.call('LOAD','FILE','',['/root/ros_ws/src/finite-state-machine---security-robot/topological_map/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D1'])
    
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D1'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D2'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D3'])
    print(" C1's doors ")
    
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D2'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D4'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D8'])
    print(" C2's doors ")
    
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C3', 'D3'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C3', 'D8'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C3', 'D6'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C3', 'D7'])
    print(" C3's doors ")
    
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R1', 'D4'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R2', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R3', 'D6'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R4', 'D7'])
    print(" rooms' doors ")
    # disjoint the room
    client.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','C3','D1','D2','D3','D4','D5','D6','D7','D8'])
    print(" disjointed ")
    
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
    client.call('REASON','','',[''])
    
    room_list = ['C1','C2','C3','R1','R2','R3','R4']
    visited(room_list)
    
    client.call('SAVE','','',['/root/ros_ws/src/finite-state-machine---security-robot/topological_map/new_map.owl'])
    
    print (" Map ready ")
    #while not rospy.is_shutdown():
    pub.publish(True)
    

if __name__ == '__main__':
    """
    the entry point of the node
    """
    try:
    	loading()
    except rospy.ROSInterruptException:
    	pass
