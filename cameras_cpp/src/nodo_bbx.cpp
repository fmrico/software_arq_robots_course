#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{

  cv_bridge::CvImagePtr img_ptr_depth;

  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }
  
  for (const auto & box : boxes->bounding_boxes) {
    int px = (box.xmax + box.xmin) / 2;
    int py = (box.ymax + box.ymin) / 2;

    float dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
    std::cerr << box.Class << " at (" << dist << std::endl;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub(nh, "/darknet_ros/bounding_boxes", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub);

  sync_bbx.registerCallback(boost::bind(&callback_bbx, _1, _2));

  ros::spin();

  return 0;
}