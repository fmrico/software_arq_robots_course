#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <softarq_msgs/SequenceAction.h>

typedef actionlib::SimpleActionClient<softarq_msgs::SequenceAction> Client;

class MyNode
{
public:
	MyNode()
	: ac("sequence", true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	void doWork(long int until)
	{
		softarq_msgs::SequenceGoal goal;
		goal.end_limit = until;

		ac.sendGoal(goal,
				boost::bind(&MyNode::doneCb, this, _1, _2),
				Client::SimpleActiveCallback(),
				boost::bind(&MyNode::feedbackCb, this, _1));

    ROS_INFO("Request sent.");

	}

	void feedbackCb(const softarq_msgs::SequenceFeedbackConstPtr& feedback)
	{
		ROS_INFO("Current count %ld", feedback->current);
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const softarq_msgs::SequenceResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Answer: %ld", result->last);
		ros::shutdown();
	}

private:
	Client ac;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "sequence_client");
	ros::NodeHandle n;

	MyNode my_node;
	my_node.doWork(200);

	ros::spin();

	return 0;
}
