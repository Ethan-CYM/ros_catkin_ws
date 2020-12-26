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
//  MatrixXd m = MatrixXd::Random(3,3);
//  m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
//  cout << "m =" << endl << m << endl;
//  VectorXd v(3);
//  v << 1, 2, 3;
//  cout << "m * v =" << endl << m * v << endl;
//  m <<1,2,3,4,5,6,7,8,9;
//  cout << "m =" << endl << m << endl;

//  double tmpA[] = {0,1,2,3,4,5};
//  Eigen::VectorXd tmpM = Eigen::Map<Eigen::VectorXd>(&tmpA[0],6);
//  Eigen::VectorXd tmpM2 = tmpM.block(0,0,3,1);
//  cout << "tmpM =" << endl << tmpM << endl;
//  cout << "tmpM2 =" << endl << tmpM2 << endl;

//  Eigen::Matrix<double,6,1> a;
//  Eigen::Matrix<double,6,1> b;
//  a<<1,2,3,4,5,6;
//  b<<11,12,13,14,15,16;
//  Eigen::Matrix<double,2,3> state;
//  state << a.block(0,0,3,1).transpose(),b.block(0,0,3,1).transpose();
//  cout << "state =" << endl << state.block(1,0,1,3) << endl;
}

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "test");
//  ros::NodeHandle n;
//  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//  ros::spin();

//  return 0;
//}
