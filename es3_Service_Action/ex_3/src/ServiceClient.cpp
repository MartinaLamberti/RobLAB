#include <ros/ros.h>
#include <cstdlib>
#include <moveit_msgs/GetPositionFK.h>

int main(int argc, char **argv) { //IL CLIENT DEVE DARE IN INPUT LE 6 VARIABILI DI GIUNTO. IL SERVER GLI RESTITUISCE LA POS E ORIENT DELL'END EFF
ros::init(argc, argv, "Forward_kinematics_COMAU_client");
if (argc != 7) { //nome del file + 6 giunti
ROS_INFO("There aren't enough params.");
return 1;
}

ros::NodeHandle n;
ros::ServiceClient client = n.serviceClient<moveit_msgs::GetPositionFK>("Forward_kinematics_COMAU");
moveit_msgs::GetPositionFK service;
service.request.header.frame_id = "base_link";
service.request.fk_link_names.resize(1);
service.request.fk_link_names[0] = "link6";
service.request.robot_state.joint_state.name.resize(6);
std::vector<std::string> joint_names {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
service.request.robot_state.joint_state.position.resize(6);
for (int i=0; i<6; i++){
service.request.robot_state.joint_state.position[i] = std::stof(argv[i+1]);
service.request.robot_state.joint_state.name[i] = joint_names[i];
}

if (client.call(service)) {
ROS_INFO("POSITION: x: %.3f - y: %.3f - z: %.3f", service.response.pose_stamped[0].pose.position.x, service.response.pose_stamped[0].pose.position.y, service.response.pose_stamped[0].pose.position.z);
ROS_INFO("QUATERNION: x: %.3f - y: %.3f - z: %.3f - w: %.3f", service.response.pose_stamped[0].pose.orientation.x, service.response.pose_stamped[0].pose.orientation.y, service.response.pose_stamped[0].pose.orientation.z, service.response.pose_stamped[0].pose.orientation.w); 
} else {
ROS_ERROR("Failed to call service Forward_Kinematics");
return 1;
}
return 0;
} 

