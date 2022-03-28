
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

#ifndef BEHAVIOR_TREES_BTACTION_H
#define BEHAVIOR_TREES_BTACTION_H

#include <ros/ros.h>

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

namespace behavior_trees
{
class BTNavAction : public BT::ActionNodeBase
{
public:
  BTNavAction(
    const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config),
    action_client_(action_name, true)
  {
    ROS_INFO("Waiting for action server to start.");
		action_client_.waitForServer();
		ROS_INFO("Action server started, sending goal.");

    nav_need_send_goal_ = true;;
    nav_finished_ = false;
    nav_succeded_ = false;
  }

  virtual void on_halt() = 0;
  virtual void on_start() {};
  virtual void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {};
  virtual BT::NodeStatus on_tick() = 0;

  void halt() 
  {
    on_halt();

    action_client_.cancelAllGoals();

    // Clean for next tick
    nav_need_send_goal_ = true;
    nav_finished_ = false;
    nav_succeded_ = false;
  }

  void set_goal(const move_base_msgs::MoveBaseGoal & new_goal)
  {
    goal_ = new_goal;
    nav_need_send_goal_ = true;
  }

  BT::NodeStatus tick()
  {
    if (status() == BT::NodeStatus::IDLE)
    {
      on_start();
    }

    if (nav_need_send_goal_)
    {
      // action_client_.cancelGoal();
      std::cerr << "Sending new goal --------------------------" << std::endl;
      action_client_.sendGoal(
        goal_,
 		  	boost::bind(&BTNavAction::doneCb, this, _1, _2),
		  	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
		  	boost::bind(&BTNavAction::feedbackCb, this, _1));
      
      nav_need_send_goal_ = false;
    }

    auto tick_status = on_tick();
    if (tick_status != BT::NodeStatus::RUNNING) {
      action_client_.cancelAllGoals();
      nav_need_send_goal_ = true;
      nav_finished_ = false;
      return tick_status;
    }

    if (nav_finished_)
    {
      if (nav_succeded_) {
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
    on_feedback(feedback);
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
	{
    nav_finished_ = true;
    nav_succeded_ = state.state_ == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED;
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
	}

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  move_base_msgs::MoveBaseGoal goal_;

  bool nav_need_send_goal_;
  bool nav_finished_;
  bool nav_succeded_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_BTACTION_H
