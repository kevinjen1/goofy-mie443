#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

namespace goofy{
namespace depth{

cv::Size GAUS_KER = cv::Size(10,10);
float IR_THRESHOLD = 100;

cv::Mat depth_image;
cv::Mat ir_image;

bool updated_image;
bool updated_ir;

void depthCallback(const sensor_msgs::ImageConstPtr msg){
  //convert the image to an openCV mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_image = cv_ptr->image;
  updated_image = true;

  return;
}

void irCallback(const sensor_msgs::ImageConstPtr msg){
  //conver the image to an OpenCV Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
  ir_image = cv_ptr->image;
  ir_image.convertTo(ir_image, CV_32FC1);
  updated_ir = true;
  return;
}

void processIR(cv::Mat& ir_image){
  //blur image
  cv::Mat blurred_image;
  cv::GaussianBlur(ir_image, blurred_image, GAUS_KER, 0, 0);
  ir_image = blurred_image;
  return;
}

void processDepth(cv::Mat& depth_image, cv::Mat& ir_image){
  //ir mask with one's and zero's, ones being where the ir image is brighter than threshold
  cv::Mat mask_bright = (ir_image > IR_THRESHOLD);
  mask_bright.convertTo(mask_bright, CV_32FC1);
  mask_bright /= 255;

  //ir mask with dark parts only (for inf)
  cv::Mat mask_dark = 1 - mask_bright;

  //depth mask with the nans filterd out, comparing against itself does this
  cv::Mat nan_mask = cv::Mat(depth_image != depth_image);
  nan_mask.convertTo(nan_mask, CV_32FC1);
  nan_mask /= 255;

  return;
}

}
}

using namespace goofy::depth;

int main(int argc, char** argv){
  ros::init(argc, argv, "depth_publisher");
  ros::NodeHandle nh;
  ros::Subscriber depth_image_callback = nh.subscribe("/camera/depth_registered/hw_registered/image_rect", 11, &goofy::depth::depthCallback);
  ros::Subscriber ir_image_callback = nh.subscribe("/camera/ir/image_rect_ir", 1, &goofy::depth::irCallback);
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
