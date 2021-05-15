// Copyright 2021 Intelligent Robotics Lab
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

#ifndef TESTING__DUPLICATOR_HPP
#define TESTING__DUPLICATOR_HPP

#include "ros/ros.h"
#include "std_msgs/Float32.h"

namespace testing
{

class Duplicator
{
public:
  Duplicator();

protected:
  void messageCallback(const std_msgs::Float32::ConstPtr& msg);
  float duplicate(float value);

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace testing

#endif  // TESTING__DUPLICATOR_HPP