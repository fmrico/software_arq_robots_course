#include "ros/ros.h"
#include "softarq_msgs/Point.h"

class Listener
{
public:
	Listener(): n_()
	{
		sub_ = n_.subscribe("/point", 1, &Listener::messageCallback, this);
	}

	void messageCallback(const softarq_msgs::Point::ConstPtr& msg)
	{
		ROS_INFO("point: (%lf, %lf, %lf)", msg->x, msg->y, msg->z);
	}

private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
};

int main(int argc, char **argv)
{

   ros::init(argc, argv, "num_subscriber");
   ros::NodeHandle n;

   Listener listener;

   ros::spin();

   return 0;

 }
