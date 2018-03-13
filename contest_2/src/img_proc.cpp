#include "func_header.h"

using namespace cv;
using namespace cv::xfeatures2d;

int findPic(imageTransporter imgTransport, vector<cv::Mat> imgs_track){
	//cv::namedWindow("view");
  	int foundPic = 0;
  
  	cv::Mat video;

	video = imgTransport.getImg();  
  	if(!video.empty()){
  	/**
		//fill with your code

  		//-- Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
  		int minHessian = 400;
  		Ptr<SURF> detector = SURF::create(minHessian);
  		vector<KeyPoint> keypoints_object, keypoints_scene;
  		Mat descriptors_object, descriptors_scene;

  		Mat img_object = imgs_track[0];
  		//Mat img_scene = video;
  		int numOfMatches [imgs_track.size()];

  		for (int im = 0; im < imgs_track.size(); im++) {
			detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
			detector->detectAndCompute(video, Mat(), keypoints_scene, descriptors_scene);

			//-- Step 3: Matching descriptor vectors using FLANN matcher
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match( descriptors_object, descriptors_scene, matches );
			double max_dist = 0; double min_dist = 100;

			//-- Quick calculation of max and min distances between keypoints
			for(int i = 0; i < descriptors_object.rows; i++){
				double dist = matches[i].distance;
				if( dist < min_dist ) min_dist = dist;
				if( dist > max_dist ) max_dist = dist;
			}
			//std::cout << "-- Max dist : " << max_dist << std::endl;
			//std::cout << "-- Min dist : " << min_dist << std::endl;

			//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
			std::vector< DMatch > good_matches;
			for( int i = 0; i < descriptors_object.rows; i++){
				if (matches[i].distance < 3*min_dist){
					good_matches.push_back( matches[i]);
				}
			}
			numOfMatches[im] = good_matches.size();
  		}

  		int bestMatch = 0;
  		for (int i=0; i<imgs_track.size(); i++) {
  			if (numOfMatches[i] > bestMatch) {
  				foundPic = i+1;
  				bestMatch = numOfMatches[i];
  			}
  		}
  		
  		std::cout << "NumOfMatches: " << bestMatch << std::endl;
  		std::cout << "Pic is:" << foundPic << std::endl;

//  		if (bestMatch < minNumOfMatches) {
//  			foundPic = 0;
//  		}
		
		//imshow("view", video);
		**/
		video.release();
		std::cout << "Video released" << std::endl;
  	} else {
  	    std::cout << "no picture" << std::endl;
  		return -1; // Can't grab image from the video
  		
  	}
  	
  	std::cout << "returning from image" << std::endl;
  	return foundPic;
}

float getArea(std::vector<Point2f> points){
	float area = 0;
	for (std::vector<Point2f>::iterator i = points.begin(); i != points.end()-1; i++){
		area += -(*i).y * (*(i+1)).x + (*i).x * (*(i+1)).y;
//		std::cout << *i << std::endl;
//		std::cout << *(i+1) << std::endl;
//		std::cout << -(*i).y * (*(i+1)).x + (*i).x * (*(i+1)).y << std::endl;
//		std::cout << area << std::endl;
	}
	area += -(*(points.end()-1)).y * (*(points.begin())).x + (*(points.end()-1)).x * (*(points.begin())).y;
//	std::cout << *(points.begin()) << std::endl;
//	std::cout << *(points.end()-1) << std::endl;
//	std::cout << area << std::endl;
	area = 0.5 * std::abs(area);
	return area;
}
