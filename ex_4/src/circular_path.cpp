#include <ros/package.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <angles/angles.h>
#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>

void computeVectorsDifference(
    Eigen::VectorXd &diff,
    const Eigen::VectorXd &minuend,
    const Eigen::VectorXd &subtrahend,
    const moveit::core::JointModelGroup *jmg)
{
    for (int i = 0; i < minuend.size(); i++)
    {
        if (jmg->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE) // Se il giunto è rotoidale
        {
            // Compute difference between revolute joints
            robot_model::VariableBounds bounds = jmg->getParentModel().getVariableBounds(jmg->getVariableNames()[i]); //prelevo i limiti di giunto

            if (bounds.position_bounded_)
            {
                angles::shortest_angular_distance_with_limits( // calcolo la minima dist angolare tenendo conto dei limiti di giunto
                    angles::normalize_angle(subtrahend[i]),
                    angles::normalize_angle(minuend[i]),
                    angles::normalize_angle(bounds.min_position_),
                    angles::normalize_angle(bounds.max_position_),
                    diff[i]);
            }
            else
                diff[i] = angles::shortest_angular_distance(subtrahend[i], minuend[i]);
        }
        else if (jmg->getActiveJointModels()[i]->getType() == robot_model::JointModel::PLANAR)
        {
            ROS_ERROR("Planar joints are not currently supported.");
        }
        else
        {
            // ! Other joint model types are included here (PRISMATIC, FIXED, UNKNOWN, etc.)
            diff[i] = minuend[i] - subtrahend[i];
        }
    }
}

int main(int argc, char **argv)
{
    // Initializing the node and the move_group interface

    ros::init(argc, argv, "circle_traj_planner");
    ros::NodeHandle node_handle;

    /*
     // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.

     * The async spinner spawns a new thread in charge of calling callbacks
     * when needed. Uncomment the lines below if, as instance, you need to
     * ask for the robot state and expect an answer before the time expires.
     */

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // Get the planning group
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("smartsix");

    // Compose trajectory path
    std::string trajectory_path = ros::package::getPath("ex_4") + "/circular_workspace.traj"; //path del file

    // Load workspace trajectory and adjust duration
    moveit_dp_redundancy_resolution::WorkspaceTrajectory ws_trajectory("yz_circular", trajectory_path); //come argomenti name e traj_path
    //preleviamo i singoli waypoint (COORDINATE E TIMESTAMP) che stanno sulla traiettoria.
    moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory ws_trajectory_msg;
    ws_trajectory.getWorkspaceTrajectoryMsg(ws_trajectory_msg); // a partire dalla traj nello spazio operativo, otteniamo il messaggio ros da pubblicare su RViz

    ROS_INFO_STREAM("Trajectory duration is " << ws_trajectory.getDuration() << " s");

    // Publish workspace trajectory to be visualized in RViz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    ros::Duration sleeping(15);
    sleeping.sleep();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(ws_trajectory.getWaypoints(), rvt::LIME_GREEN, rvt::SMALL); // per tenere traccia della traiettoria che il robot deve seguire
    visual_tools.trigger();

    // Create empty joint-space trajectory
    robot_trajectory::RobotTrajectory robot_trajectory(kinematic_model, joint_model_group);
    // traiettoria nello spazio giunti vuota da riempire con IK dei punti che stanno nella traj dello sp operativo

    // Compute IK and verify limits
    double dt = 0;
    Eigen::VectorXd joint_positions_prev = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_positions_curr = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_velocities_prev = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_velocities_curr = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_accelerations = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());

    for (int i = 0; i < ws_trajectory_msg.timestamps.size(); i++) // for per ogni punto della traiettoria nello spazio di lavoro.
    {
        moveit::core::RobotState robot_state(kinematic_model);

        if (!robot_state.setFromIK(joint_model_group, ws_trajectory_msg.waypoints[i])) // calcolo della cinematica inversa per ottenere la traj nello spazio giunti
            ROS_WARN_STREAM("Could not compute IK solution for waypoint " << i);

        if (i > 0)
        {
            dt = ws_trajectory_msg.timestamps[i] - ws_trajectory_msg.timestamps[i - 1]; // per tutti i punti tranne il primo si computano le diff tra element corr e prec per calcolare il rapporto incrementale

            robot_state.copyJointGroupPositions(joint_model_group, joint_positions_curr);

            computeVectorsDifference(joint_velocities_curr, joint_positions_curr, joint_positions_prev, joint_model_group);
            joint_velocities_curr = joint_velocities_curr / dt;

            joint_accelerations = (joint_velocities_curr - joint_velocities_prev) / dt;

            robot_state.setJointGroupVelocities(joint_model_group, joint_velocities_curr);
            robot_state.setJointGroupAccelerations(joint_model_group, joint_accelerations);
        }

        robot_trajectory.addSuffixWayPoint(robot_state, dt); // aggiunge il punto alla traj nello spazio giunti

        joint_positions_prev = joint_positions_curr;
        joint_velocities_prev = joint_velocities_curr;
    }

    moveit_msgs::RobotTrajectory robot_trajectory_msg;

    robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg); // traj nello spazio giunti convertita in messaggio ros per portarla in RViz

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish the joint space solution");

    // Prepare display trajectory message and publish it
    // PUBLISHER per pubblicare verso RViz e quindi visualizzare la nostra soluzione cinematica e quindi la traiettoria spazio giunti
    ros::Publisher display_path_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/" + planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);
    std::cout << planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC << std::endl;
    // Viene pubblicato un messaggio di tipo DisplayTrajectory.
    moveit_msgs::DisplayTrajectory display_trajectory_msg;

    display_trajectory_msg.model_id = kinematic_model->getName();
    display_trajectory_msg.trajectory.resize(1, robot_trajectory_msg);
    robot_state::robotStateToRobotStateMsg(robot_trajectory.getFirstWayPoint(), display_trajectory_msg.trajectory_start); // Convert a MoveIt! robot state to a robot state message
    display_path_publisher.publish(display_trajectory_msg);

    /*
    # The model id for which this path has been generated
    # The representation of the path contains position values for all the joints that are moving along the path; a sequence of trajectories may be specified
    # The robot state is used to obtain positions for all/some of the joints of the robot. 
    # It is used by the path display node to determine the positions of the joints that are not specified in the joint path message above. 
    # If the robot state message contains joint position information for joints that are also mentioned in the joint path message, the positions in the joint path message will overwrite the positions specified in the robot state message. 

    */

    // * Ottenere la traj nello spazio di lavoro da matlab
    // * conversione della traj nello spazio giunti ( tramite IK)
    // * calcolo delle vel e acc nello spazio giunti (tramite il rapporto incrementale)
    // * dalla traj dello spazio giunti creo un publisher per pubblicare verso RViz e quindi visualizzare la nostra soluzione cinematica e quindi la traiettoria spazio giunti

    // FINE PUNTO 3

    // PUNTO 4: Viene creato un altro publisher per avere un plot della traiettoria (che facciamo con rqt multi plot)

    // Publish joint space solution to plot in rqt_multiplot
    ros::Publisher plot_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("plot_planned_trajectory", 10000, true);
    // il tipo di messaggio pubblicato è JointTrajectoryPoint, ovvero l'insieme dei punti della traiettoria nello spazio giunti

    ros::Duration sleep_time(0.05);

    for (int i = 0; i < robot_trajectory_msg.joint_trajectory.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint jtp = robot_trajectory_msg.joint_trajectory.points[i];

        plot_trajectory_publisher.publish(jtp); //pubblicazione di ogni punto JointTrajectoryPoint

        sleep_time.sleep(); // un po' di delay altrimenti li pubblicherebbe troppo velocemente
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to terminate the planner");

    spinner.stop();
    ros::shutdown();
    exit(0);
}
