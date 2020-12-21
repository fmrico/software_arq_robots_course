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


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

enum {IDX_h, IDX_H, IDX_s, IDX_S, IDX_v, IDX_V, NUM_HSV};

typedef struct
{
  float hsv[NUM_HSV];
  ros::Publisher hsv_pubs[NUM_HSV];
}
HSVInfo;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int hupper_, hlower_;
  int supper_, slower_;
  int vupper_, vlower_;
  int channel_;

  static const int MAX_CHANNELS = 3;
  HSVInfo hsvValues_[MAX_CHANNELS];

public:
  ImageConverter(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/hsv/image_filtered", 1);

    cv::namedWindow("Imagen Fuente");
    cv::namedWindow("Imagen filtrada");

    // TrackBar
    cv::createTrackbar("Hue Upper", "Imagen filtrada", &hupper_, 360, NULL);
    cv::createTrackbar("Hue Lower", "Imagen filtrada", &hlower_, 360, NULL);
    cv::createTrackbar("Sat Upper", "Imagen filtrada", &supper_, 255, NULL);
    cv::createTrackbar("Sat Lower", "Imagen filtrada", &slower_, 255, NULL);
    cv::createTrackbar("Val Upper", "Imagen filtrada", &vupper_, 255, NULL);
    cv::createTrackbar("Val Lower", "Imagen filtrada", &vlower_, 255, NULL);
    cv::createTrackbar("Channel", "Imagen filtrada", &channel_, MAX_CHANNELS-1, NULL);
    cv::setTrackbarPos("Hue Upper", "Imagen filtrada", 360);
    cv::setTrackbarPos("Hue Lower", "Imagen filtrada", 0);
    cv::setTrackbarPos("Sat Upper", "Imagen filtrada", 255);
    cv::setTrackbarPos("Sat Lower", "Imagen filtrada", 0);
    cv::setTrackbarPos("Val Upper", "Imagen filtrada", 255);
    cv::setTrackbarPos("Val Lower", "Imagen filtrada", 0);
    cv::setTrackbarPos("Channel", "Imagen filtrada", 0);

    for (int i=0; i < MAX_CHANNELS; i++)
      initChannel(&hsvValues_[i], i);

    publishHSV();
  }

  ~ImageConverter()
  {
    // cv::destroyWindow("Imagen Fuente");
    // cv::destroyWindow("Imagen filtrada");
  }

  void initChannel(HSVInfo * hsvinfo, int i)
  {
    hsvinfo->hsv[IDX_h] = 0;
    hsvinfo->hsv[IDX_H] = 360;
    hsvinfo->hsv[IDX_s] = 0;
    hsvinfo->hsv[IDX_S] = 255;
    hsvinfo->hsv[IDX_v] = 0;
    hsvinfo->hsv[IDX_V] = 255;

    char topic_id[256];

    sprintf(topic_id, "/hsv_filter/%d/h", i);
    hsvinfo->hsv_pubs[IDX_h] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
    sprintf(topic_id, "/hsv_filter/%d/H", i);
    hsvinfo->hsv_pubs[IDX_H] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);

    sprintf(topic_id, "/hsv_filter/%d/s", i);
    hsvinfo->hsv_pubs[IDX_s] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
    sprintf(topic_id, "/hsv_filter/%d/S", i);
    hsvinfo->hsv_pubs[IDX_S] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);

    sprintf(topic_id, "/hsv_filter/%d/v", i);
    hsvinfo->hsv_pubs[IDX_v] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
    sprintf(topic_id, "/hsv_filter/%d/V", i);
    hsvinfo->hsv_pubs[IDX_V] = nh_.advertise<std_msgs::Float32>(topic_id, 1, true);
  }

  void imageCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    bool changed = (hsvValues_[channel_].hsv[IDX_h] != hlower_) || (hsvValues_[channel_].hsv[IDX_H] != hupper_) ||
        (hsvValues_[channel_].hsv[IDX_s] != slower_) || (hsvValues_[channel_].hsv[IDX_S] != supper_) ||
        (hsvValues_[channel_].hsv[IDX_v] != vlower_) || (hsvValues_[channel_].hsv[IDX_V] != vupper_);

    hsvValues_[channel_].hsv[IDX_h] = hlower_;
    hsvValues_[channel_].hsv[IDX_H] = hupper_;
    hsvValues_[channel_].hsv[IDX_s] = slower_;
    hsvValues_[channel_].hsv[IDX_S] = supper_;
    hsvValues_[channel_].hsv[IDX_v] = vlower_;
    hsvValues_[channel_].hsv[IDX_V] = vupper_;

    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_imageout = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

    int height = hsv.rows;
    int width = hsv.cols;
    int step = hsv.step;
    int channels = 3;  // RGB


    for (int i=0; i < height; i++ )
      for (int j=0; j < width; j++ )
      {
        int posdata = i * step + j * channels;

        if (!((hsv.data[posdata] >= hsvValues_[channel_].hsv[IDX_h]) &&
             (hsv.data[posdata] <= hsvValues_[channel_].hsv[IDX_H]) &&
            (hsv.data[posdata+1] >= hsvValues_[channel_].hsv[IDX_s]) &&
             (hsv.data[posdata+1] <= hsvValues_[channel_].hsv[IDX_S]) &&
            (hsv.data[posdata+2] >= hsvValues_[channel_].hsv[IDX_v]) &&
             (hsv.data[posdata+2] <= hsvValues_[channel_].hsv[IDX_V])))
        {
          cv_imageout->image.data[posdata] = 0;
          cv_imageout->image.data[posdata+1] = 0;
          cv_imageout->image.data[posdata+2] = 0;
        }
      }


    if (changed)
      publishHSV();

    // Update GUI Window
    cv::imshow("Imagen Fuente", cv_ptr->image);
    cv::imshow("Imagen filtrada", cv_imageout->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_imageout->toImageMsg());
  }

  void publishHSV()
  {
    for (int i=0; i < MAX_CHANNELS; i++)
    {
      std_msgs::Float32 msg;

      msg.data = hsvValues_[i].hsv[IDX_h];
      hsvValues_[i].hsv_pubs[IDX_h].publish(msg);
      msg.data = hsvValues_[i].hsv[IDX_H];
      hsvValues_[i].hsv_pubs[IDX_H].publish(msg);

      msg.data = hsvValues_[i].hsv[IDX_s];
      hsvValues_[i].hsv_pubs[IDX_s].publish(msg);
      msg.data = hsvValues_[i].hsv[IDX_S];
      hsvValues_[i].hsv_pubs[IDX_S].publish(msg);

      msg.data = hsvValues_[i].hsv[IDX_v];
      hsvValues_[i].hsv_pubs[IDX_v].publish(msg);
      msg.data = hsvValues_[i].hsv[IDX_V];
      hsvValues_[i].hsv_pubs[IDX_V].publish(msg);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hsv_tuner_2d");
  ImageConverter ic;
  ros::spin();
  return 0;
}
