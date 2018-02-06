#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<limits>
#include<chrono>

namespace goofy{
namespace depth{

cv::Size GAUS_KER = cv::Size(11,11);
float IR_THRESHOLD = 30;

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
  // std::cout << "Blurring image" << std::endl;

  cv::Mat blurred_image;
  cv::GaussianBlur(ir_image, blurred_image, GAUS_KER, 0, 0);
  ir_image = blurred_image;

  // imwrite("./blurred_image.jpg", ir_image);

  return;
}

void processDepth(cv::Mat& depth_image, const cv::Mat& ir_image){
  //std::cout << "Masking images" << std::endl;

  //std::cout << ir_image << std::endl;

  //ir mask with one's and zero's, ones being where the ir image is brighter than threshold
  cv::Mat mask_bright = (ir_image > IR_THRESHOLD);

  //ir mask with dark parts only (for inf)
  cv::Mat mask_dark = (ir_image < IR_THRESHOLD);

  //depth mask with the nans only out where 1s are nans, comparing against itself does this
  cv::Mat nan_mask = cv::Mat(depth_image != depth_image);

  // imwrite("./bright_mask.jpg", mask_bright);
  // imwrite("./dark_mask.jpg", mask_dark);
  //
  // imwrite("./orig_depth.jpg", depth_image);

  //make copies of it
  cv::Mat nan_mask_bright = nan_mask;
  cv::Mat nan_mask_dark = nan_mask;

  nan_mask_bright.mul(mask_bright);
  nan_mask_dark.mul(mask_dark);

  // std::cout << "Creating nans" << std::endl;

  cv::Mat global_inf(depth_image.size(), CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));
  cv::Mat global_nan(depth_image.size(), CV_32FC1, cv::Scalar(nanf("")));

  // std::cout << "Copying values" << std::endl;

  global_inf.copyTo(depth_image, nan_mask_bright); // dark pixels and nan depth_registered
  global_nan.copyTo(depth_image, nan_mask_dark); // bright pixels and nan depth registered

  // imwrite("./new_depth.jpg", depth_image);

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
  ros::Publisher depth_publisher = nh.advertise<sensor_msgs::Image>("/corrected_depth", 1);

  //initialize updates to false
  updated_image = false;
  updated_ir = false;

  while (ros::ok()){
    ros::spinOnce();
    if (updated_image == true & updated_ir == true){

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        processIR(ir_image);
        processDepth(depth_image, ir_image);

        cv_bridge::CvImage depth_msg;
        depth_msg.image = depth_image;
        depth_msg.encoding = "32FC1";
        depth_publisher.publish(depth_msg.toImageMsg());

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::chrono::milliseconds processing = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << "Processing time: " << processing.count() << std::endl;

        updated_image = false;
        updated_ir = false;
    }
  }

}
