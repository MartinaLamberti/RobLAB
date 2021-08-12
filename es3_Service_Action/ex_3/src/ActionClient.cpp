#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <custom_action_msgs/IK_ex3Action.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <string>

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const custom_action_msgs::IK_ex3ResultConstPtr & result);
void handleGoalActiveEvent();
void handleFeedbackEvent(const custom_action_msgs::IK_ex3FeedbackConstPtr & feedback);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Action_Client");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<custom_action_msgs::IK_ex3Action> ac("Action_IK", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    custom_action_msgs::IK_ex3Goal goal;
    goal.pos_quaternion_endeff.position.x = 1.0; // valori di posizione  dell'end eff per cui si vuole conoscere la cin inversa
    goal.pos_quaternion_endeff.position.y = 0.0;
    goal.pos_quaternion_endeff.position.z = 1.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);                                // definisco il quaternione a partire dagli angoli di roll pitch e yaw.
    goal.pos_quaternion_endeff.orientation = tf2::toMsg(q); //trasformazione di una variabile quaternione in un messaggio quaternione
    ac.sendGoal(goal, &handleGoalCompletionEvent, &handleGoalActiveEvent, &handleFeedbackEvent);

    if (!ac.waitForResult(ros::Duration(30.0)))
        ROS_ERROR("Action did not finish before the time out.");

    //exit
    return 0;
}

// CALLBACK QUANDO IL SERVER HA COMPLETATO L'AZIONE.
void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState &state, const custom_action_msgs::IK_ex3ResultConstPtr &result)
{
    std::ostringstream ss;

    if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) //controllo dello stato di completamento se SUCCEDEED
    {
        int num_of_solutions = result->all_conf_joints.size(); // size del vettore che contiene tutte le soluzioni
       
        ROS_INFO("The Action Client starts to print the received solutions");
        ROS_INFO("Goal achieved --- ALL SOLUTIONS ARE:");
        for (int i = 0; i < num_of_solutions; i++)
        {
            ROS_INFO("SOLUTION #%d", i);
            for (int j = 0; j < result->all_conf_joints[i].position.size(); j++) // da 0 a 5 visto che sono 6 giunti
                ROS_INFO("JOINT #%d: %f", j, result->all_conf_joints[i].position[j]);
        }

        //VISUALIZZAZIONE DELLE SOLUZIONI IN RVIZ
        ros::NodeHandle nh; 

        // Instantiate the joint state publisher publishing on the joint_states topic  --NODO PUBLISHER CHE PUBBLICA JOINT STATE SUL TOPIC JOINT STATES
        ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

        //CARICAMENTO DATI DAL PARAMETER SERVER in modo tale da comunicare con il robot in esecuzione
        // Load the robot model
        robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

        // Get the robot kinematic model
        robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();


        // Get the planning group
        const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("smartsix");

        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = joint_model_group->getVariableNames();

        ROS_INFO("Publishing solutions...");

        ros::Duration sleep_time(2.0);

        for (int i = 0; i < num_of_solutions; i++) //pubblicazione delle posizioni di giunto così RViz si colloca lì
        {
            sleep_time.sleep();

            joint_state_msg.position = result->all_conf_joints[i].position;
            joint_state_msg.header.stamp = ros::Time::now();
            
            joint_state_publisher.publish(joint_state_msg);
        }

        ROS_INFO("All solutions published");
    }
    else ROS_INFO("Goal aborted. %s", state.getText() );

}

void handleGoalActiveEvent()
{
    ROS_INFO("Inverse kinematics request sent to the IK resolution action server");
}


void handleFeedbackEvent(const custom_action_msgs::IK_ex3FeedbackConstPtr &feedback)
{

    ROS_INFO("FEEDBACK --- Received IK solution: ");
    for (int j = 0; j < feedback->joints.position.size(); j++) // da 0 a 5 visto che sono 6 giunti
                ROS_INFO("JOINT #%d: %f", j, feedback->joints.position[j]);

}
