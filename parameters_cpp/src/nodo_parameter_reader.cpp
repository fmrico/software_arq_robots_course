#include "ros/ros.h"

class ParameterReader
{
public:
	ParameterReader(): nh_("~")
	{
    std::string my_name;
    std::string the_greeting;

    nh_.getParam("name", my_name);
    the_greeting = nh_.param("greeting", std::string("Hello!!"));
    nh_.setParam("/message", the_greeting + " from " + my_name);

    ROS_INFO("Message from node parameter_reader: %s from %s", the_greeting.c_str(), my_name.c_str());
	}

private:
	ros::NodeHandle nh_;
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "parameter_reader");

   ParameterReader parameter_reader;

   ros::spin();

   return 0;

 }
