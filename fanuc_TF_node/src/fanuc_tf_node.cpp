#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>

void printRotations (const geometry_msgs::TransformStamped transformStamp){
    tf2::Quaternion q(transformStamp.transform.rotation.x, transformStamp.transform.rotation.y, transformStamp.transform.rotation.z, transformStamp.transform.rotation.w);
    tf2::Vector3 axis = q.getAxis(); // Return the axis of the rotation represented by this quaternion (valore di ritorno è Vector3).
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    printf("Computing the Rotation matrix...\n");
    printf("[ %f, %f, %f ]\n", m[0][0], m[0][1], m[0][2]);
    printf("[ %f, %f, %f ]\n", m[1][0], m[1][1], m[1][2]);
    printf("[ %f, %f, %f ]\n\n", m[2][0], m[2][1], m[2][2]);
    printf("Computing the Euler angles...\n");
    printf("[ %f, %f, %f ]\n\n", roll, pitch, yaw);
    printf("Computing the axis-angle representation...\n");
    printf("Axis = [%f, %f, %f]\n", axis.x(), axis.y(), axis.z());
    printf("Angle = %f\n\n", q.getAngle());
    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  const char* links[8] = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6", "flange"}; 
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
        for (int i=0; i<=6 ; i++){
                  transformStamped = tfBuffer.lookupTransform(links[i], links[7], ros::Time(0));
            printf (" TRANSLATION: %f %f %f \n ", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z); 
            printf ("--------------- %s to Flange -------------------- \n", links[i]); 
            printRotations (transformStamped);
        }

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
};