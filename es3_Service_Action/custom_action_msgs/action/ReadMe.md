This package was created in order to customize the action and the communication between action client and action server.

IK_ex3.action contains 3 Messages separated by 3 dashes:

Goal: the goal that the server should try to reach.

Result: when the server terminates (either success, failure, or abortion), it will send a result to the client. 

Feedback: at any time during the execution, the server can send some feedback to the client. 

The Goal is a variable geometry_msgs/Pose that memorizes position and orientation of the end effector, in order to obtain the joints configurations.

The Feedback is a variable sensor_msgs/JointState that is sent by the server during the execution that contains a new solution, i.e. the joints configuration to reach that Pose.

The Result is a vector of JointState that collects, at the end of the compilation, all the solution achieved by the IK that includes the joints variables.



