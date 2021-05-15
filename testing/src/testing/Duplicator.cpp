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

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "testing/Duplicator.hpp"

namespace testing
{

Duplicator::Duplicator()
: n_()
{
  sub_ = n_.subscribe("/input", 100, &Duplicator::messageCallback, this);
  pub_=n_.advertise<std_msgs::Float32>("/output", 100);
}

void
Duplicator::messageCallback(const std_msgs::Float32::ConstPtr& msg)
{
  std_msgs::Float32 response_msg;
  response_msg.data = duplicate(msg->data);

  pub_.publish(response_msg);
}

float
Duplicator::duplicate(float value)
{
  return 2.0f * value;
}

}  // namespace testing
