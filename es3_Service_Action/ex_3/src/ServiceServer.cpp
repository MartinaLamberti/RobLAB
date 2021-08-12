#include <ros/ros.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>


bool forward_kinematics(moveit_msgs::GetPositionFK::Request &request, moveit_msgs::GetPositionFK::Response &response)
{
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("smartsix");
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

kinematic_state->setJointGroupPositions(joint_model_group, request.robot_state.joint_state.position);
const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link6"); //info da inviare al client: end_effector_state

response.pose_stamped.resize(1); //"conversioni" da end_effector_state a pose_stamped.
Eigen::Translation<double, 3> t(end_effector_state.translation());
response.pose_stamped[0].pose.position.x = t.x();
response.pose_stamped[0].pose.position.y = t.y();
response.pose_stamped[0].pose.position.z = t.z();
Eigen::Quaternion<double> q(end_effector_state.rotation());
response.pose_stamped[0].pose.orientation.x = q.x();
response.pose_stamped[0].pose.orientation.y = q.y();
response.pose_stamped[0].pose.orientation.z = q.z();
response.pose_stamped[0].pose.orientation.w = q.w();

/* Print end-effector pose. Remember that this is in the model frame */
ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");


return true;

}

int main(int argc, char **argv) //crea un servizio che si mette in ascolto e quando riceve la richiesta getPositionFK chiama la funzione forward kinematics.
{
ros::init(argc, argv, "Forward_kinematics_COMAU_server");
ros::NodeHandle nh;
ros::ServiceServer service = nh.advertiseService("Forward_kinematics_COMAU", forward_kinematics);
ros::spin();
return 0;
}

/*
# The resultant vector of PoseStamped messages that contains the (stamped) poses of the requested links
geometry_msgs/PoseStamped[] pose_stamped

# The list of link names corresponding to the poses
string[] fk_link_names

MoveItErrorCodes error_code

*/


