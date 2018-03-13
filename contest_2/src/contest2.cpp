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

//-------------------------move robot function---------------
bool moveToGoal(float xGoal, float yGoal, float phiGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
    	geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}
}



Cereal::Cereal (int one, int two) {
	coord = one;
	logo = two;
}

Status::Status(int one, bool two) {
	coord = one;
	success = two;
}

/**
 * Change the variable 'coordIndex' to point an explored position
 * Returns true if unexplored position was found. Otherwise false
 */
bool getNextCoord(int* coordIndex, int count, vector<Status> mission) {
	int startingIndex = *coordIndex;
	int index = startingIndex;
	
	std::cout << "count:" << count << std::endl;

	do {
		index = (index+1)%count;
		std::cout << "indexUpdate:" << index << std::endl;
	} while (index != startingIndex || mission[index].success);

	if (index == startingIndex) {
		if (mission[index].success) {
			return false; // successfully completed all missions
		}
	}
	*coordIndex = index;
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<cv::Mat> imgs_track;	
	vector<Cereal> mappedPics;
	vector<Status> mission;
	
	vector<vector<std::string> > filenames;
	filenames.resize(5, std::vector<std::string>(3));
	filenames[0][0] = "./c0_im1.jpg";
	filenames[0][1] = "./c0_im2.jpg";
	filenames[0][2] = "./c0_im3.jpg";
	
	filenames[1][0] = "./c1_im1.jpg";
	filenames[1][1] = "./c1_im2.jpg";
	filenames[1][2] = "./c1_im3.jpg";
	
	filenames[2][0] = "./c2_im1.jpg";
	filenames[2][1] = "./c2_im2.jpg";
	filenames[2][2] = "./c2_im3.jpg";
	
	filenames[3][0] = "./c3_im1.jpg";
	filenames[3][1] = "./c3_im2.jpg";
	filenames[3][2] = "./c3_im3.jpg";
	
	filenames[4][0] = "./c4_im1.jpg";
	filenames[4][1] = "./c4_im2.jpg";
	filenames[4][2] = "./c4_im3.jpg";
	
	vector<std::string> logoNames;
	logoNames.push_back("Blank");
	logoNames.push_back("Raisin Bran");
	logoNames.push_back("Cinnamon Toast Crunch");
	logoNames.push_back("Rice Krispies");

	if(!init(coord, imgs_track)) return 0;

	int count = coord.size();

	for(int i = 0; i < count; ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
		mission.push_back(Status(i,false));
	}

	// imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect

    // THIS SHOULD BE ZERO
	int coordIndex = 0;
	
	int truePic[5] = {1,1,3,2,0};
	vector<vector<int> > Pic;
	Pic.push_back(vector<int> (5));
	int iter = 0;

    ros::spinOnce();
    vector<float> beginCoord;
    beginCoord.push_back(x);
    beginCoord.push_back(y);
    beginCoord.push_back(phi);
    

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    	//...................................

    	//fill with your code
        //std::cout << "Starting while loop" << std::endl;
   		bool isMovedToPosition = false;
   		// figure out the coordinates of the robot's desired position, given coordinates of boxes (coord[coordIndex])
   		// then pass them to moveToGoal

   		vector<float> currCoord = coord[coordIndex];
   		float curr_x = currCoord[0] + DIST * std::cos(currCoord[2]);
   		
   		//std::cout << std::cos(currCoord[2]) << std::endl;
   		//std::cout << std::sin(currCoord[2]) << std::endl;
   		//std::cout << DIST << std::endl;
   		
   		//std::cout << currCoord[0] << std::endl;
   		
   		float curr_y = currCoord[1] + DIST * std::sin(currCoord[2]);

   		//std::cout << "Sending position x: " << curr_x << " y: " << curr_y << " angle: " << currCoord[2] << std::endl;

   		isMovedToPosition = moveToGoal(curr_x, curr_y, currCoord[2] - PI);
   		
   		//std::cout << "Exited move to goal" << std::endl;

   		if (isMovedToPosition) {
		    //std::cout << "Got to position" << std::endl;
   			ros::Duration(2).sleep(); // wait to ensure robot has settled
   			ros::spinOnce();
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
          		double areaOfBoxes [imgs_track.size()];
          		double minArea = 1000;
          		

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
              		
              		//std::cout << "matches: " << good_matches.size() << " - inliers:" << inlierSum << std::endl;
              		//std::cout << "ratio: " << inlierRatio << std::endl;
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
              		//-- Show detected matches
              		// imshow( "Good Matches & Object detection", img_matches );
              		
              		
              		//-- Show detected matches
              		//imshow( "Good Matches & Object detection", img_matches );
              		
              		//std::cout << "outputting file: " << filenames[coordIndex][im] << std::endl;
              		std::string filename = filenames[coordIndex][im];
              		imwrite(filename, img_matches);
              		//std::cout << "checkFile:" << checkFile << std::endl;
              		
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

                    
                    float area = getAreaGoofy(scene_corners);
                    //int area = (maxX - minX) * (maxY - minY);
                    //std::cout << "RECTANGLE " << minX << " " << maxX << " " << minY << " " << maxY << std::endl;
                    std::cout << "area: " << area << std::endl;
                    areaOfBoxes[im] = area;
                    
                    float topM = -1.0f;
                  float lowM = -1.0f;
                  float leftM = -1.0f;
                  float rightM = -1.0f;

                  bool isHorzParallel = false;
                  bool isVertParallel = false;

                  // Horz Slope
                  float runTop = abs(scene_corners[1].x - scene_corners[0].x);
                  float riseTop = abs(scene_corners[1].y - scene_corners[0].y);

                  if (runTop != 0)
                  {
                    topM = riseTop / runTop;
                  }

                  float runLow = abs(scene_corners[2].x - scene_corners[3].x);
                  float riseLow = abs(scene_corners[2].y - scene_corners[3].y);
                  
                  if (runLow != 0)
                  {
                    lowM = riseLow / runLow;
                  }

                  // Vert Slope - Flip
                  float runLeft = abs(scene_corners[0].x - scene_corners[3].x);
                  float riseLeft = abs(scene_corners[0].y - scene_corners[3].y);

                  if (riseLeft != 0)
                  {
                    leftM = runLeft / riseLeft;
                  }

                  float runRight = abs(scene_corners[1].x - scene_corners[2].x);
                  float riseRight = abs(scene_corners[1].y - scene_corners[2].y);
                  
                  if (riseRight != 0)
                  {
                    rightM = runRight / riseRight;
                  }

                  std::cout << "Slopes: Top - " << topM << " Bottom - " << lowM << " Left - " << leftM << " Right - " << rightM << std::endl;
                  if (topM < 0.5 && lowM < 0.5 && leftM < 0.5 && rightM < 0.5)
                  {
                  std::cout << "VALID LOGO" << std::endl;
                  } 
          		}

                int bestMatch = 0;
          		int picMatch = 0;
          		double bestRatio = 0;
          		int picRatio = 0;
          		double bestArea = 0;
          		int picRect = 0;
          		
          		for (int i=0; i<imgs_track.size(); i++) {
          			if (numOfMatches[i] > bestMatch) {
          				bestMatch = i+1;
          				bestMatch = numOfMatches[i];
          			}
          			if (ratioOfInliers[i] > bestRatio) {
          			    picRatio = i+1;
          			    bestRatio = ratioOfInliers[i];
          			}
          			if (areaOfBoxes[i] > bestArea) {
          			    picRect = i+1;
          			    bestArea = areaOfBoxes[i];
          			}
          		}
          		
          		//std::cout << "pictu gets: " << foundPic << " - NumOfMatches: " << bestMatch << std::endl;
			    //std::cout << "ratio gets: " << picRatio << " - BestRatio: " << bestRatio << std::endl;
			    std::cout << "green gets: " << picRect << " - BestArea: " << bestArea << std::endl;
			    
			    if (bestArea < minArea) {
			        std::cout << "this is a white sqaure" << std::endl;
			        picRect = 0;
			    }
				foundPic = picRect;
				
  		
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

   		if (!isMoreToGo) {
   		    std::cout << "I'm done everything!" << std::endl;
			int check[4] = {0,0,0,0};
			
			
			for(int b=0; b<count; b++) {
			    std::cout << "Coordinate " << b << ": (" << coord[b][0] << ", " << coord[b][1] << ", " << coord[b][2] << ")" << std::endl;
				std::cout << " -- logo: " << Pic[iter][b] << " - " << logoNames[Pic[iter][b]] << std::endl;
				check[Pic[iter][b]]++;
			}
			// The robot has found a valid configuration
			if(check[0]>0 && check[1]>0 && check[2]>0 && check[3] == 1) {
					moveToGoal(beginCoord[0], beginCoord[1], beginCoord[2]);
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

float getArea(std::vector<Point2f> points){
	float area = 0;
	for (std::vector<Point2f>::iterator i = points.begin(); i != points.end(); i++){
		area += -(*i).y * (*(i+1)).x + (*i).x * (*(i+1)).y;
	}
	area += -(*(points.end())).y * (*(points.begin())).x + (*(points.end())).x * (*(points.begin())).y;
	area = 0.5 * std::abs(area);
	return area;
}
