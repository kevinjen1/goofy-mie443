#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <func_header.h>

#include <eStop.h>
#include <string>

using namespace cv;
using namespace cv::xfeatures2d;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static const float DIST = 0.7;
static const float PI = 3.14;

float x;
float y;
float phi;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    	x = msg.pose.pose.position.x;
    	y = msg.pose.pose.position.y;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "image_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<cv::Mat> imgs_track;	

	if(!init(coord, imgs_track)) return 0;

	int count = coord.size();

	for(int i = 0; i < count; ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
	}

	// imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect

	int coordIndex = 0;
	
	int truePic[5] = {1,1,3,2,0};
	vector<vector<int> > Pic;
	Pic.push_back(vector<int> (5));
	int iter = 0;
	vector<std::string> filenames;
	filenames.push_back("./file1.jpg");
	filenames.push_back("./file2.jpg");
	filenames.push_back("./file3.jpg");
	filenames.push_back("./file4.jpg");
	filenames.push_back("./file5.jpg");
	
	vector<std::string> croppedfilenames;
	croppedfilenames.push_back("./filecrop1.jpg");
	croppedfilenames.push_back("./filecrop2.jpg");
	croppedfilenames.push_back("./filecrop3.jpg");
	croppedfilenames.push_back("./filecrop4.jpg");
	croppedfilenames.push_back("./filecrop5.jpg");

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    	//...................................

   		bool isMovedToPosition = true;
   		

   		if (isMovedToPosition) {
		    //std::cout << "Got to position" << std::endl;
   			ros::Duration(2).sleep(); // wait to ensure robot has settled
   			//int fPic = findPic(imgTransport, imgs_track); NOT USED. COPIED EVERYTHING HERE INSTEAD
   			cv::Mat video = imgTransport.getImg();
   			int foundPic = 0;
   			
   			std::cout << "True picture: " << truePic[coordIndex] << std::endl;
   			
   			if(!video.empty()){
   			    //fill with your code

          		//-- Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
          		int minHessian = 400;
          		Ptr<SURF> detector = SURF::create(minHessian);
          		vector<KeyPoint> keypoints_object, keypoints_scene;
          		Mat descriptors_object, descriptors_scene;

          		
          		//Mat img_scene = video;
          		int numOfMatches [imgs_track.size()];
          		double ratioOfInliers [imgs_track.size()];
          		

          		for (int im = 0; im < imgs_track.size(); im++) {
					Mat img_object = imgs_track[im];
          		   	std::vector< DMatch > good_matches;
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
			        //std::vector< DMatch > good_matches;
			        for( int i = 0; i < descriptors_object.rows; i++){
				        if (matches[i].distance < 3*min_dist){
					        good_matches.push_back( matches[i]);
				        }
			        }
			        numOfMatches[im] = good_matches.size();
			        
			        //-- Localize the object
              		std::vector<Point2f> obj;
              		std::vector<Point2f> scene;
              		for(int i = 0; i < good_matches.size(); i++){
			            //-- Get the keypoints from the good matches
			            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
			            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
              		}
                    Mat mask;
              		Mat H;
              		H = findHomography( obj, scene, RANSAC, 3, mask);
              		cv::Size s = mask.size();
              		int n = s.height;
              		int inlierSum = cv::sum(mask)[0];
              		double inlierRatio = ((double)inlierSum)/n;
              		ratioOfInliers[im] = inlierRatio;
              		
              		std::cout << "matches: " << good_matches.size() << " - inliers:" << inlierSum << std::endl;
              		std::cout << "ratio: " << inlierRatio << std::endl;
              		Mat img_matches;
              		drawMatches( img_object, keypoints_object, video, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
              		
              		//-- Get the corners from the image_1 ( the object to be "detected" )
              		std::vector<Point2f> obj_corners(4);
              		obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
              		obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
              		std::vector<Point2f> scene_corners(4);
              		perspectiveTransform( obj_corners, scene_corners, H);

              		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
              		line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0),
              		scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
              		line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0),
              		scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
              		line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0),
              		scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
              		line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0),
              		scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
              		
              		
              		std::cout << "outputting file: " << filenames[im] << std::endl;
              		//-- Show detected matches
              		//imshow( "Good Matches & Object detection", img_matches );
              		std::string filename = filenames[im];
              		bool checkFile = imwrite(filename, img_matches);
              		std::cout << "checkFile:" << checkFile << std::endl;
              		
              		//ros::Duration(5).sleep(); // wait to ensure robot has settled
              		
              		// Histogram Comparison - Prepare Video Image
                  int minX = video.cols;
                  int maxX = 0;
                  int minY = video.rows;
                  int maxY = 0;
                  for (int i = 0; i < scene_corners.size(); i++)
                  {
                    if (scene_corners[i].x > maxX)
                    {
                      maxX = scene_corners[i].x;
                    }
                    if (scene_corners[i].x < minX)
                    {
                      minX = scene_corners[i].x;
                    }
                    if (scene_corners[i].y > maxY)
                    {
                      maxY = scene_corners[i].y;
                    }
                    if (scene_corners[i].y < minY)
                    {
                      minY = scene_corners[i].y;
                    }
                  }
                  
                  int area = (maxX - minX) * (maxY - minY);
                   std::cout << "RECTANGLE " << minX << " " << maxX << " " << minY << " " << maxY << std::endl;
                   std::cout << "AREA: " << area << std::endl;
                   /*if (maxX > minX && maxY > minY)
                   {
                  Rect myROI(minX, minY, maxX, maxY);
                  Mat croppedVideo = video(myROI);


                    std::cout << "outputting file: " << croppedfilenames[im] << std::endl;
              		//-- Show detected matches
              		std::string cropfilename = croppedfilenames[im];
              		bool checkCropFile = imwrite(cropfilename, croppedVideo);
              		std::cout << "checkFile:" << checkCropFile << std::endl;
                  // Histogram Comparison - Prepare Histograms
                  Mat hsv_video;
                  Mat hsv_logo;

                  //cvtColor( croppedVideo, hsv_video, CV_BGR2HSV );
                  //cvtColor( img_object, hsv_logo, CV_BGR2HSV );

                  int h_bins = 50;
                  int s_bins = 60;
                  int histSize[] = {h_bins, s_bins};
                  float h_range[] = {0, 180};
                  float s_range[] = {0, 256};
                  const float * ranges[] = {h_range, s_range};
                  int channels[] = {0, 1};
                  MatND histLogo;
                  MatND histVideo;

                  calcHist( &croppedVideo, 1, channels, Mat(), histVideo, 2, histSize, ranges, true, false );
                  normalize( histVideo, histVideo, 0, 1, NORM_MINMAX, -1, Mat() );

                  calcHist( &img_object, 1, channels, Mat(), histLogo, 2, histSize, ranges, true, false );
                  normalize( histLogo, histLogo, 0, 1, NORM_MINMAX, -1, Mat() );

                  for( int i = 0; i < 4; i++ )
                     { 
                      int compare_method = i;
                      double comparePerfect = cv::compareHist(histVideo, histLogo, compare_method);
                      double compareHist = cv::compareHist( histVideo, histLogo, compare_method );
                      
                      std::cout << "Method: " << i << " Perfect: " << comparePerfect << " Video vs Logo: " << compareHist << std::endl; 
                    }
                    }*/
  		
          		}

          		int bestMatch = 0;
          		double bestRatio = 0;
          		int picRatio = 0;
          		for (int i=0; i<imgs_track.size(); i++) {
          			if (numOfMatches[i] > bestMatch) {
          				foundPic = i+1;
          				bestMatch = numOfMatches[i];
          			}
          			if (ratioOfInliers[i] > bestRatio) {
          			    picRatio = i+1;
          			    bestRatio = ratioOfInliers[i];
          			}
          		}
          		
          		std::cout << "pictu gets: " << foundPic << " - NumOfMatches: " << bestMatch << std::endl;
			    std::cout << "ratio gets: " << picRatio << " - BestRatio: " << bestRatio << std::endl;
				

  		
   			    //std::cout << "releasting video" << endl;
   			    video.release();
   			} else {
   			    std::cout << "video is empty" << endl;
   			}
   			//std::cout << "exited function" << std::endl;
   			if (foundPic >= 0) {
				Pic[iter][coordIndex] = foundPic;
   			    //std::cout << "Got picture" << std::endl;
   				//mappedPics.push_back(Cereal (coordIndex, foundPic));
   				//mission[coordIndex].success = true; NOT USED
   				//std::cout << "Pic is:" << fPic << std::endl; NOT USED
   			}
   			
   		} else {
   		    std::cout << "Can't get to position" << std::endl;
   			//figure out something
   		}
   		//bool isMoreToGo = getNextCoord(&coordIndex, count, mission);
   		coordIndex++;
   		bool isMoreToGo = false;
   		if (coordIndex < count) {
   		    isMoreToGo = true;
   		}
   		
   		//std::cout << "newCoordIndex:" << coordIndex << std::endl;
   		//std::cout << "isMoreToGo: " << isMoreToGo << std::endl;
   		
   		// THIS MAKES IT RUN ONCE
   		break;
   		
   		
   		if (!isMoreToGo) {
   		    std::cout << "I'm done everything!" << std::endl;
			int check[4] = {0,0,0,0};
			for(int b=0; b<count; b++) {
				std::cout << "Coordinate: " << (b+1) << " logo: " << Pic[iter][b] << std::endl;
				check[Pic[iter][b]]++;
			}
			// The robot has found a valid configuration
			if(check[0]>0 && check[1]>0 && check[2]>0 && check[3] == 1) {
					break;
			}
			// The robot has not found a valid configuration, do another run
			else {
				iter++;
				Pic.push_back(vector<int> (5));
				coordIndex = 0;
			}
   			// display logo for each coordinate
   			// go back to beginning. Sing a lullaby. Do a victory dance.
   			
   		}
   		//std::cout << "I'm going to the next point" << std::endl;
   		//ros::Duration(2).sleep(); // wait to ensure robot has settled

	}
	return 0;
}
