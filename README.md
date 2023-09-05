# finite state machine - security robot

*Introduction*

This repository is a ROS package that contains a software architecture for a surveillance robot.
The implementation of the architecture is done with a finite state machine for the core behaviour of the robot.
Inside the package are also present two other nodes: one for the creation of the ontology map, and the other is a simulation of the battery of the robot.
The programming language used is Python3.
SMACH tool is used for the implementation of the state machine (for more information see https://wiki.ros.org/smach).
aRMOR service is at the base of the whole architecture (https://github.com/EmaroLab/armor).
The map is created as a OWL document and the reasoning is done trhough Protegé.
