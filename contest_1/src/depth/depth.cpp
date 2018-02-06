#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

namespace goofy{
namespace depth{

cv::Mat depth_image;
cv::Mat ir_image;

bool updated_image;
bool updated_ir;

void depthCallback(const sensor_msgs::ImageConstPtr msg){
  //convert the image to an openCV mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  depth_image = cv_ptr->image;
  updated_image = true;
  return;
}

void irCallback(const sensor_msgs::ImageConstPtr msg){
  //conver the image to an OpenCV Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  ir_image = cv_ptr->image;
  updated_ir = true;
  return;
}

void processIR(cv::Mat& ir_image){
  return;
}

void processDepth(cv::Mat& depth_image, cv::Mat& ir_image){
  return;
}

}
}

using namespace goofy::depth;

int main(int argc, char** argv){
  ros::init(argc, argv, "depth_publisher");
  ros::NodeHandle nh;
  ros::Subscriber depth_image_callback = nh.subscribe("/depth_image", 11, &goofy::depth::depthCallback);
  ros::Subscriber ir_image_callback = nh.subscribe("/ir_image", 1, &goofy::depth::irCallback);
  ros::Publisher rearranged_depth_callback = nh.advertise<sensor_msgs::Image>("/corrected_depth", 1);

  //initialize updates to false
  updated_image = false;
  updated_ir = false;

  while (ros::ok()){
    ros::spinOnce();
    if (updated_image == true & updated_ir == true){
        processIR(ir_image);
        processDepth(ir_image, depth_image);
    }
  }

}
