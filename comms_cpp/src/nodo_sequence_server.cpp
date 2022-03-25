#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <softarq_msgs/SequenceAction.h>

class SequenceAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<softarq_msgs::SequenceAction> as_;
  std::string action_name_;

  softarq_msgs::SequenceFeedback feedback_;
  softarq_msgs::SequenceResult result_;

public:

  SequenceAction(std::string name) :
    as_(nh_, name, boost::bind(&SequenceAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    ROS_INFO("Server Ready");
  }

  ~SequenceAction(void)
  {
  }

  void executeCB(const softarq_msgs::SequenceGoalConstPtr &goal)
  {
    ROS_INFO("Server Ready");
    ros::Rate r(10);
    bool success = true;


    feedback_.current = 0;

    ROS_INFO("%s: Executing, creating sequence until %ld", action_name_.c_str(), goal->end_limit);

    for(int i=1; i<=goal->end_limit; i++)
    {

      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
        break;
      }

      feedback_.current++;

      as_.publishFeedback(feedback_);

      r.sleep();
    }

    if(success)
    {
      result_.last = feedback_.current;
      ROS_INFO("%s: Succeeded", action_name_.c_str());

      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sequence");

  SequenceAction fibonacci(ros::this_node::getName());
  ros::spin();

  return 0;
}
