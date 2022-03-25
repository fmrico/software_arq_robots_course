#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class MyNode
{
public:
	MyNode()
	: ac("move_base", true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	void doWork(long int until)
	{
		move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 3.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending action");
		ac.sendGoal(goal,
				boost::bind(&MyNode::doneCb, this, _1, _2),
				Client::SimpleActiveCallback(),
				boost::bind(&MyNode::feedbackCb, this, _1));

    ROS_INFO("Action sent");

	}

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ros::shutdown();
	}

private:
	Client ac;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "client_nav");
	ros::NodeHandle n;

	MyNode my_node;
	my_node.doWork(200);

	ros::spin();

	return 0;
}
