
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

#include <string>

#include "behavior_trees/Turn.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace behavior_trees
{

Turn::Turn(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

void
Turn::halt()
{
  ROS_INFO("finished turning");
}

BT::NodeStatus
Turn::tick()
{
  ros::Time turn_ts_;
  if(getInput<ros::Time>("object2").has_value()){
    turn_ts_ = getInput<ros::Time>("object2").value();
  }
  geometry_msgs::Twist cmd;
  cmd.angular.z = TURNING_VEL;
  pub_vel_.publish(cmd);
  if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        ROS_INFO("TURNING -> GOING_FORWARD");
        return BT::NodeStatus::SUCCESS;
      }
    else  
    {
      return BT::NodeStatus::RUNNING;
    }
}

}  // namespace behavior_trees
