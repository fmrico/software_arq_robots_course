#include "ros/ros.h"
#include "softarq_msgs/Point.h"

int main(int argc, char **argv)
{

   ros::init(argc, argv, "point_publisher");
   ros::NodeHandle n;

   ros::Publisher point_pub = n.advertise<softarq_msgs::Point>("/point", 1);

   ros::Rate loop_rate(10);

   while (ros::ok())
   {
	   softarq_msgs::Point msg;

	   msg.x = 8.0;
	   msg.y = 9.0;
	   msg.z = 10.0;

	   point_pub.publish(msg);

	   ros::spinOnce();
	   loop_rate.sleep();
   }

   return 0;

 }
