
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

#ifndef BEHAVIOR_TREES_FORWARD_H
#define BEHAVIOR_TREES_FORWARD_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include <string>

namespace behavior_trees
{

class Forward : public BT::ActionNodeBase
{
  public:
    ros::NodeHandle n_;
    explicit Forward(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();
    static BT::PortsList providedPorts(){};

  private:
    static constexpr float GOING_FORWARD_VEL = 0.2;
    ros::Publisher pub_vel_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_FORWARD_H
