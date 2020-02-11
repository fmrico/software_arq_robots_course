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

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "geometry_tf/transforms.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_sub");
  ros::NodeHandle n;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    geometry_msgs::TransformStamped bf2odom_msg;
    geometry_msgs::TransformStamped odom2bf_msg;
    geometry_msgs::TransformStamped odom2obj_msg;
    geometry_msgs::TransformStamped obj2odom_msg;

    tf2::Stamped<tf2::Transform> bf2odom;
    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::Stamped<tf2::Transform> odom2obj;
    tf2::Stamped<tf2::Transform> obj2odom;

    tf2::Transform bf2obj;
    tf2::Transform obj2bf;

    std::string error;

    if (buffer.canTransform("base_footprint", "odom", ros::Time(0), ros::Duration(0.1), &error))
    {
      bf2odom_msg = buffer.lookupTransform("base_footprint", "odom", ros::Time(0));

      tf2::fromMsg(bf2odom_msg, bf2odom);

      double roll, pitch, yaw;
      tf2::Matrix3x3(bf2odom.getRotation()).getRPY(roll, pitch, yaw);

      ROS_INFO("base_footprint -> odom [%lf, %lf, %lf] %lf ago",
        bf2odom.getOrigin().x(),
        bf2odom.getOrigin().y(),
        yaw,
        (ros::Time::now() - bf2odom.stamp_).toSec());
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    if (buffer.canTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.1), &error))
    {
      odom2bf_msg = buffer.lookupTransform("odom", "base_footprint", ros::Time(0));

      tf2::fromMsg(odom2bf_msg, odom2bf);

      double roll, pitch, yaw;
      tf2::Matrix3x3(odom2bf.getRotation()).getRPY(roll, pitch, yaw);

      tf2::fromMsg(odom2obj_msg, odom2obj);
      ROS_INFO("odom -> base_footprint  [%lf, %lf, %lf] %lf ago",
        odom2bf.getOrigin().x(),
        odom2bf.getOrigin().y(),
        yaw,
        (ros::Time::now() - odom2bf.stamp_).toSec());
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    if (buffer.canTransform("odom", "object", ros::Time(0), ros::Duration(0.1), &error))
    {
      odom2obj_msg = buffer.lookupTransform("odom", "object", ros::Time(0));

      tf2::fromMsg(odom2obj_msg, odom2obj);

      double roll, pitch, yaw;
      tf2::Matrix3x3(odom2obj.getRotation()).getRPY(roll, pitch, yaw);

      ROS_INFO("odom -> object [%lf, %lf, %lf] %lf ago",
        odom2obj.getOrigin().x(),
        odom2obj.getOrigin().y(),
        yaw,
        (ros::Time::now() - odom2obj.stamp_).toSec());
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    if (buffer.canTransform("object", "odom", ros::Time(0), ros::Duration(0.1), &error))
    {
      obj2odom_msg = buffer.lookupTransform("object", "odom", ros::Time(0));

      tf2::fromMsg(obj2odom_msg, obj2odom);

      double roll, pitch, yaw;
      tf2::Matrix3x3(odom2obj.getRotation()).getRPY(roll, pitch, yaw);

      ROS_INFO("object -> odom  [%lf, %lf, %lf] %lf ago",
        obj2odom.getOrigin().x(),
        obj2odom.getOrigin().y(),
        yaw,
        (ros::Time::now()- obj2odom.stamp_).toSec());
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    bf2obj = bf2odom * odom2obj;

    ROS_INFO("base_footprint -> object  [%lf, %lf, %lf] %lf ago",
      bf2obj.getOrigin().x(), bf2obj.getOrigin().y(), bf2obj.getOrigin().z(),
      (ros::Time::now()-odom2obj.stamp_).toSec());

    tf2::Vector3 p_obj(1.0, 0.0, 0.0);
    tf2::Vector3 p_bf = geometry_tf::transform_point(p_obj, bf2obj);

    ROS_INFO("Point [%lf, %lf, %lf] in object frame is [%lf, %lf, %lf] in basefootprint",
      p_obj.x(), p_obj.y(), p_obj.z(),
      p_bf.x(), p_bf.y(), p_bf.z());

  ROS_INFO("===========================================================================");

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
