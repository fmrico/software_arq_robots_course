#include "ros/ros.h"
#include "softarq_msgs/Distance.h"

 #include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_client");
  if (argc != 7)
  {
    ROS_INFO("usage: distance_client x1 y1 z1 x2 y2 z2");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<softarq_msgs::Distance>("distance");

  ROS_INFO("Waiting for server");
  while (!client.waitForExistence(ros::Duration(1.0))) {
    ROS_INFO("Still waiting");
  }
  ROS_INFO("Server ready");

  softarq_msgs::Distance srv;

  srv.request.a.x = atof(argv[1]);
  srv.request.a.y = atof(argv[2]);
  srv.request.a.z = atof(argv[3]);
  srv.request.b.x = atof(argv[4]);
  srv.request.b.y = atof(argv[5]);
  srv.request.b.z = atof(argv[6]);
  
  if (client.call(srv))
  {
    ROS_INFO("Distance: %lf", srv.response.distance);
  }
  else
  {
    ROS_ERROR("Failed to call service distance");
    return 1;
  }

  return 0;
}

