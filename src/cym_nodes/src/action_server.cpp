#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cym_nodes/ActionMessageAction.h>

int count = 0;
int count_failure = false;

class ActionServer{
private:
  ros::NodeHandle n_;
  actionlib::SimpleActionServer<cym_nodes::ActionMessageAction> ac_;
  cym_nodes::ActionMessageGoal goal_;
  cym_nodes::ActionMessageResult result_;
  cym_nodes::ActionMessageFeedback feedback_;

public:
  ActionServer();
  ~ActionServer(void){}


  void executeCB(const actionlib::SimpleActionServer
                 <cym_nodes::ActionMessageAction>::GoalConstPtr &goal);
};

int main(int argc, char **argv)
{

}
