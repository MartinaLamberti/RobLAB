In this folder there are:
- ServiceServer
- ServiceClient
- ActionServer
- ActionClient

The Service Server computes the forward kinematics of a robot and the Service Client prints the solution to stdout. 

--FOR THE SERVICE

COMANDI:

1° TERMINALE--- When in roblab_ws:
 source devel/setup.bash 
 roslaunch smartsix_moveit_config demo.launch 

 2° TERMINALE---
 source devel/setup.bash 
 rosrun ex_3 ServiceServer_node 


3° TERMINALE---
source devel/setup.bash 
rosrun ex_3 ServiceClient_node 0 0 0 0 0 0 where these 6 args are the six joint variables. In this way, the server can calculate the positiotion and orientation of end effector.

-- FOR THE ACTION

COMANDI:

1° TERMINALE--- When in roblab_ws:
 source devel/setup.bash 
 roslaunch smartsix_moveit_config demo.launch 

2° TERMINALE---
 source devel/setup.bash 
 rosrun ex_3 ActionServer_node
 catkin build ex_3


3° TERMINALE---
 source devel/setup.bash 
 rosrun ex_3 ActionClient_node 
