#include <Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}



using namespace Eigen;
using namespace std;

int main()
{
  MatrixXd m = MatrixXd::Random(3,3);
  m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  cout << "m =" << endl << m << endl;
  VectorXd v(3);
  v << 1, 2, 3;
  cout << "m * v =" << endl << m * v << endl;
}

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "test");
//  ros::NodeHandle n;
//  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//  ros::spin();

//  return 0;
//}
