# *finite state machine - security robot*

# *Introduction* 

This repository is a ROS package that contains a software architecture for a surveillance robot. 
The implementation of the architecture is done with a finite state machine for the core behaviour of the robot.
Inside the package are also present two other nodes: one for the creation of the ontology map, and the other is a simulation of the battery of the robot. 
The programming language used is Python3.
SMACH tool is used for the implementation of the state machine (for more information see https://wiki.ros.org/smach).
aRMOR service is at the base of the whole architecture (https://github.com/EmaroLab/armor).
The map is created as a OWL document and the reasoning is done trhough Protegé.

# *Architecture* 

# The map
This node uses the aRMOR client in order to upload and modify an ontology already provived (https://github.com/buoncubi/topological_map). 
In the original ontology are present the classes: `ROBOT`,`DOOR` and `LOCATION` that has three subclasses `ROOM`, `CORRIDOR` and `URGENT`; the object proprieties; the data proprieties and the SWLRules that are used to infer the knoledge trhogh Protegé.
The `ROOM` is a `LOCATION` with only one `DOOR`, the `CORRIDOR` is a `LOCATION` with at least two `DOOR` and a `LOCATION` became `URGENT` we is not visited for at least 7 seconds. 
The map node creates five instances for the class ROOM ['E','R1','R2','R3','R4'] that are connected with three CORRIDOR ['C1','C2','C3']. 
The 'E' room represent the initial and the charging room and is connected only with the 'C1' corridor.
The 'C1' corridor is connected with the others two corridors 'C2', 'C3' that have doors in common, respectivily, with 'R1', 'R2' and 'R3', 'R4'. 
This node subscribe to the boolean `"/load"` topic to indicate that full creation of the map. <br>
This is the environment created by the node in which the robot moves
![ontology image](https://github.com/tommasodeangeli97/finite-state-machine---security-robot/assets/92479113/3c194e24-fa89-4283-a30a-aa5d9a630170)

# The battery
This is a simple Publish/Subscribe node used to simulate the status of the battery. 
The node subscribe to the boolean `"/room"` topic that indicates the presence of the robot inside the 'E' room in order to "charge the battery" (it waits a certain amount of second), once that the battery is full the node publish *False* on the boolean `"/batttrigger"` topic;
when the robot leaves the 'E' room the code waits a random time before indicating that the battery is low publishing *True* on the `"/batttrigger"` topic.  

# The state machine
This is the node where the behaviour of the robot is created as it's shown in this video


https://github.com/tommasodeangeli97/finite-state-machine---security-robot/assets/92479113/d758c4fd-9dac-4cda-bd89-f6f9d4a813fb

The state machine is created with four state <br>
* RECEIVING_MAP: In this state the robot waits for the map to be uploaded, it listen to the boolean topic `"/load"` and once it became *True* it change status in `PATROLLING_CORRIDOR`.
* PATROLLING_CORRIDOR: the robot goes trhough the various corridors and preferes to stay in corridors untill a room becames `URGENT`; when a urgent room is triggered the robot valuates if that room is accessible, if it is so it goes to the room and changes status in `PATROLLING_ROOM`, if it is not it searches for the best path to reach the indicated room, it remains in the `PATROLLING_CORRIDOR` status untill the room is reached; if there are more then one urgent room, the robot chooses the nearest room and preferes to go there. <br>
The robot listen to the topic `"/batttrigger"` and in the moment that it becames *True* it changes status going in `BATTERY_LOW`.
* PATROLLING_ROOM: the robot is in this status when he has already reached a room; it listen to the `"/batttrigger"` topic, if the value of `"/batttrigger"` becames *True* the robot change status in `BATTERY_LOW`; it waits few seconds and goes back to the nearest corridor, it changes it's status in `PATROLLING_CORRIDOR`.
* BATTERY_LOW: wherever the robot is when `"/batttrigger"` becames *True* the status changes to this status; the robot choose the best path to reach the 'E' room, once it is in the 'E' room it publish *True* on the `"/room"` topic; it waits untill the `"/batttrigger"` value becames *False* ,remainig in the `BATTERY_LOW` status, and restart it's journey changing to the `PATROLLING_CORRIDOR` status and publishing *False* on the `"/room"` topic.
  
  ![graph](https://github.com/tommasodeangeli97/finite-state-machine---security-robot/assets/92479113/b94ba172-11f1-40da-9e2f-3a087a2b8c2d)

# *Nodes communication*
In the architecture are present only three topic:
* `"/room"` : is the topic that communicates between the `state machine` node and the `battery` node, it indicates the presence of the robot inside the 'E' room (the charging room).
* `"/batttrigger"` : this topic is constantly watched in the state machine node, it indicates if the battery is full or low; this topic communicates between the `state machine` node and the `battery` node.
* `"/load"` : this topic communicates between the `map` node and the `state machine` node; it indicates the fully creation of the map.

  ![rqt graph](https://github.com/tommasodeangeli97/finite-state-machine---security-robot/assets/92479113/bd61106d-2343-4f24-901b-d1c74b847728)

# *Installation*
To run this code firstly make sure that you have correctly installed aRMOR (https://github.com/EmaroLab/armor) and SMACH (https://wiki.ros.org/smach). <br>
Insiede your ROS workspace copy the package <br>

 ` git clone https://github.com/tommasodeangeli97/finite-state-machine---security-robot/tree/main `
  
<br>
Make shure that the python nodes are executable <br>

 ` chmod +x <name_of_file.py> `

<br>
Inside the `map.py` code change the root of your package at line 95 <br>

`  client.call('LOAD','FILE','',['root_to_your_package/topological_map/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false']) `

and at line 131 <br>

`  client.call('SAVE','','',['root_to_your_package/topological_map/new_map.owl']) `

<br>
The same inside the `statemachine.py` code at line 211 <br>

`  client.call('LOAD','FILE','',['root_to_your_package/topological_map/new_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false']) `

<br>
from the terminal go inside the root of your workspace and do <br>

 ` catkin_make `

<br>
now is possible to to launch the code <br>

`  roslaunch finite-state-machine---security-robot state_machine_launcher.launch `

<br>
in order to visualize the diagram in an other terminal run <br>

 ` rosrun smach_viewer smach_viewer.py `

