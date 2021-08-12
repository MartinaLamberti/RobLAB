#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <custom_action_msgs/IK_ex3Action.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <angles/angles.h>
#include <vector>

class IK_Action
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<custom_action_msgs::IK_ex3Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    custom_action_msgs::IK_ex3Feedback feedback_;
    custom_action_msgs::IK_ex3Result result_;
    const robot_state::JointModelGroup* jmg_;
    robot_model::RobotModelPtr kinematic_model_; 
    

public:
    IK_Action(std::string name) : as_(nh_, name, boost::bind(&IK_Action::executeCB, this, _1), false),
    action_name_(name) //costruttore della classe
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description", true);
        kinematic_model_ = robot_model_loader.getModel();

        // Get the planning group
        jmg_ = kinematic_model_->getJointModelGroup("smartsix");
        as_.start();
    }

    ~IK_Action(void) //distruttore della classe
    {
    }

    std::vector<double> generateSeedState_() const
    {
        std::vector<double> seed_state; //vettore di double, seed per ogni giunto

        std::vector<std::string> joint_names = kinematic_model_->getVariableNames();

        for (int i = 0; i < joint_names.size(); i++) //per ogni giunto prendiamo il limite inf e sup
        {
            double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
            double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
            double span = ub - lb; //range utile a definire il valore di giunto interno allo span

            seed_state.push_back((double)std::rand() / RAND_MAX * span + lb); //viene preso un numero casuale nello span
        }

        return seed_state;
    }

    void normalizeJointPositions_(std::vector<double> &solution) const
    {
        for (int i = 0; i < solution.size(); i++)
        {
            if (jmg_->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
            {
                solution[i] = angles::normalize_angle(solution[i]);
            }
        }
    }

    bool checkUniqueSol_(const std::vector<double> &solution) const
    {
        for (int i = 0; i < result_.all_conf_joints.size(); i++) //si scorre tutto il vettor delle soluzioni
        {
            bool flag = true;

            for (int j = 0; j < result_.all_conf_joints[i].position.size() && flag; j++) // la j scandisce la variabile di giunto sulla soluzione i-esima
            {
                double diff;

                if (jmg_->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE) //se il giunto è rotoidale
                {
                    diff = angles::shortest_angular_distance(result_.all_conf_joints[i].position[j], solution[j]);
                    //distanza angolare più breve tra la soluzione trovata e la soluzione corrente nel vettore delle soluzioni che stiamo analizzando
                }
                else
                {
                    diff = result_.all_conf_joints[i].position[j] - solution[j];
                }

                if (std::fabs(diff) > 1e-3) // se il valore assoluto della differenza è superiore a 10^-3
                    flag = false;           //le soluzioni non matchano
            }

            if (flag) //se le soluzioni sono uguali, allora la soluzione esiste già e non bisogna inserirla.
                return false;
        }

        return true;
    }

    void executeCB(const custom_action_msgs::IK_ex3GoalConstPtr &goal) //callback che viene chiamata quando viene catturata una richiesta
    {
        //ricevuta la richiesta, bisogna calcolare la cinematica inversa.
        ROS_INFO("IK solver: goal received");

        // Joint model group solver instance
        const kinematics::KinematicsBaseConstPtr solver = jmg_->getSolverInstance(); // da joint model group ci stiamo prendendo il solver cinematico per IK.

        int i = 0;
        

        // L’ALGORITMO DI RICERCA È UNA TRASPOSIZIONE TRA LO SPAZIO GIUNTI E LO SPAZIO OPERATIVO: CALCOLO L’ERRORE NELLO SPAZIO OPERATIVO,
        // TRAMITE LO JACOBIANO MI PORTO L’ERRORE NELLO SPAZIO GIUNTI, AGGIORNO LA SOLUZIONE SPAZIO-GIUNTI, RICALCOLO LA CINEMATICA DIRETTA E VEDO DOVE SONO FINITO.
        while (i < 3000 && ros::ok()) //max 3000 volte calcoliamo l'errore.
        {
            std::vector<double> seed_state = generateSeedState_(); // seme per calcolare la configurazione iniziale nello spazio giunti per iniziare il processo di trasposizione.
            std::vector<double> solution;                          // la soluzione è costituita dai 6 angoli dei giunti
            moveit_msgs::MoveItErrorCodes error_code;

            solver->getPositionIK(goal->pos_quaternion_endeff, seed_state, solution, error_code);

            if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                normalizeJointPositions_(solution); // normalizzazione angoli da 0 a 2pi
                int num_sols = result_.all_conf_joints.size();
                if (checkUniqueSol_(solution)) // se la soluzione è nuova:
                {
                     
                    for (int k=0; k < solution.size(); k++ )
                    feedback_.joints.position.push_back(solution[k]);        //inserisco in feedback la soluzione singola
                    sensor_msgs::JointState var; 
                    for (int k=0; k < solution.size(); k++ )
                    var.position.push_back(solution[k]);
                    result_.all_conf_joints.push_back(var); //inseriamo la soluzione alla lista da inviare al richiedente
                    as_.publishFeedback(feedback_);               //viene inviato al richiedente
                    feedback_.joints.position.clear();
                }
            }
            i++;
        }

        if (result_.all_conf_joints.size() == 0)
            as_.setAborted(result_, "Could not find any IK solution");
        else
            ROS_INFO("Found %d IK solutions", result_.all_conf_joints.size());

        as_.setSucceeded(result_);
        result_.all_conf_joints.resize(0);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Action_IK");

    IK_Action ik_action("Action_IK");
    ROS_INFO("Started inverse kinematics action server");
    ros::spin();

    return 0;
}