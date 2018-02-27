#include "func_header.h"

using namespace cv::xfeatures2d;

int findPic(imageTransporter imgTransport, vector<cv::Mat> imgs_track){
	cv::namedWindow("view");
  	int foundPic;
  
  	cv::Mat video;

	video = imgTransport.getImg();  
  	if(!video.empty()){
		//fill with your code
	  
		cv::imshow("view", video);
		video.release();
  	}
  
  	return foundPic;
}
