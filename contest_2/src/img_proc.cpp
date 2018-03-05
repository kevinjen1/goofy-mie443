#include "func_header.h"

using namespace cv;
using namespace cv::xfeatures2d;

int findPic(imageTransporter imgTransport, vector<cv::Mat> imgs_track){
	cv::namedWindow("view");
  	int foundPic;
  
  	cv::Mat video;

	video = imgTransport.getImg();  
  	if(!video.empty()){
		//fill with your code

  		//-- Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
  		int minHessian = 400;
  		Ptr<SURF> detector = SURF::create(minHessian);
  		vector<KeyPoint> keypoints_object, keypoints_scene;
  		Mat descriptors_object, descriptors_scene;

  		Mat img_object = imgs_track[0];
  		Mat img_scene = video;
  		int numOfMatches [imgs_track.size()];

  		for (int im = 0; im < imgs_track.size(); im++) {
			detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
			detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);

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
			std::cout << "-- Max dist : " << max_dist << std::endl;
			std::cout << "-- Min dist : " << min_dist << std::endl;

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

//  		if (bestMatch < minNumOfMatches) {
//  			foundPic = 0;
//  		}


//  		Mat img_matches;
//  		drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//  		//-- Localize the object
//  		std::vector<Point2f> obj;
//  		std::vector<Point2f> scene;
//  		for(int i = 0; i < good_matches.size(); i++){
//			//-- Get the keypoints from the good matches
//			obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//			scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//  		}
//
//  		Mat H = findHomography( obj, scene, RANSAC );
//  		//-- Get the corners from the image_1 ( the object to be "detected" )
//  		std::vector<Point2f> obj_corners(4);
//  		obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
//  		obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
//  		std::vector<Point2f> scene_corners(4);
//  		perspectiveTransform( obj_corners, scene_corners, H);
//
//  		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
//  		line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0),
//  		scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
//  		line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0),
//  		scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  		line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0),
//  		scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  		line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0),
//  		scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  		//-- Show detected matches
//  		imshow( "Good Matches & Object detection", img_matches );
	  
		imshow("view", video);
		video.release();
  	} else {
  		return -1;
  	}
  
  	return foundPic;
}
