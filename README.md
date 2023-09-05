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
The map node cretes five instances for the class ROOM ['E','R1','R2','R3','R4'] that are connected with three CORRIDOR ['C1','C2','C3']. 
The 'E' room represent the initial and the charging room and is connected only with the 'C1' corridor.
The 'C1' corridor is connected with the others two corridors 'C2', 'C3' that have doors in common, respectivily, with 'R1', 'R2' and 'R3', 'R4'. 
This node subscribe to the boolean `"/load"` topic to indicate that full creation of the map.
![ontology image](https://github.com/tommasodeangeli97/finite-state-machine---security-robot/assets/92479113/3c194e24-fa89-4283-a30a-aa5d9a630170)

# The battery
This is a simple Publish/Subscribe node used to simulate the status of the battery.
The node subscribe to the boolean `"/room"` topic that indicates the presence of the robot inside the 'E' room in order to "charge the battery" (it waits a certain amount of second), once that the battery is full the node publish *False* on the boolean `"/battery"` topic;
when the robot leaves the 'E' room the code waits a random time before indicating that the battery is low publishing *True* on the `"/battery"` topic.  
