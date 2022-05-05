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
#include <vector>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


class RGBDFilter
{
public:
  RGBDFilter()
  {
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &RGBDFilter::cloudCB, this);
    image_sub_ = nh_.subscribe("/hsv/image_filtered", 1, &RGBDFilter::imageCB, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);

    last_image_ptr_ = nullptr;
  }


  void imageCB(const sensor_msgs::Image::ConstPtr& msg)
  {
    try
    {
      last_image_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    if (last_image_ptr_ == nullptr)
    {
      return;
    }

    auto pcrgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    auto pcrgb_out = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::fromROSMsg(*cloud_in, *pcrgb);

    int height = last_image_ptr_->image.rows;
    int width = last_image_ptr_->image.cols;
    int step = last_image_ptr_->image.step;
    int channels = 3;  // RGB

    for (int i=0; i < height; i++ )
      for (int j=0; j < width; j++ )
      {
        int posdata_img = i * step + j * channels;
        int posdata_3d = i * width + j;

        if ((last_image_ptr_->image.data[posdata_img] != 0) ||
          (last_image_ptr_->image.data[posdata_img + 1] != 0) || 
          (last_image_ptr_->image.data[posdata_img + 2] != 0))
        {
          if (!std::isnan((*pcrgb)[posdata_3d].x) && !std::isinf((*pcrgb)[posdata_3d].x))
          {
            pcrgb_out->push_back((*pcrgb)[posdata_3d]);
          }
        }
      }

    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*pcrgb_out, cloud_out);

    cloud_out.header.frame_id = cloud_in->header.frame_id;
    cloud_out.header.stamp = ros::Time::now();

    cloud_pub_.publish(cloud_out);
  }


private:

  ros::NodeHandle nh_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber image_sub_;
  ros::Publisher cloud_pub_;

  cv_bridge::CvImagePtr last_image_ptr_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_filter");
  RGBDFilter rf;
  ros::spin();
  return 0;
}
