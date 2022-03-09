
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

#include "behavior_trees/Back.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace behavior_trees
{

Back::Back(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

void
Back::halt()
{
  ROS_INFO("finished backing");
}

BT::NodeStatus
Back::tick()
{
  ros::Time press_ts_;
  if(getInput<ros::Time>("object1").has_value()){
    press_ts_ = getInput<ros::Time>("object1").value();
  }

  geometry_msgs::Twist cmd;
  cmd.linear.x = GOING_BACK_VEL;
  pub_vel_.publish(cmd);
  if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
      {
        ROS_INFO("GOING_BACK -> TURNING");
        turn_ts_ = ros::Time::now();
        setOutput<ros::Time>("object2",turn_ts_);
        return BT::NodeStatus::SUCCESS;
      }
    else  
    {
      return BT::NodeStatus::RUNNING;
    }
}

}  // namespace behavior_trees
