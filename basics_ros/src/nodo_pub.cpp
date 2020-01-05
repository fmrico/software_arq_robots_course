// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"
#include "std_msgs/Int64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "num_publisher");
  ros::NodeHandle n;

  ros::Publisher num_pub = n.advertise<std_msgs::Int64>("/message", 1);

  ros::Rate loop_rate(10);

  int count=0;

  while (ros::ok())
  {
    std_msgs::Int64 msg;
    msg.data = count++;

    num_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
 }
