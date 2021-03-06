#include "PlayVideo.hpp"
 
using namespace std;
using namespace cv;

std::string FOLDER_ROOT = ROOT_DIR;

void VideoPlayer::play(string filename){
	_stop = false;
	video_player = std::thread(&VideoPlayer::PlayVideo, this, filename);
	//std::cout << "Detached video playing thread" << std::endl;
}

void VideoPlayer::stop(){
	_stop = true;
	if (video_player.joinable()){
		//std::cout << "Joining thread" << std::endl;
		video_player.join();
	}
}

VideoPlayer::~VideoPlayer(){
	stop();
}

int VideoPlayer::PlayVideo(string filename)
{
    // Create a VideoCapture object and open the input file
    VideoCapture cap(filename + ".mp4"); 
    sound_play::SoundClient sc; 

    // Check if camera opened successfully
    if(!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        cout << filename << endl;
        return -1;
    }

    // Play Sound
    sc.playWave(filename + ".wav");
    
    int i = 0;
    // Play Video
    std::chrono::steady_clock::time_point start_time;
    Mat frame;
    while(1){
        start_time = std::chrono::steady_clock::now();
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
          break;

        //std::cout << "Displaying frame" << std::endl;

        // Display the resulting frame
        cv_bridge::CvImage image_message;
        image_message.encoding = "bgr8";
        image_message.image = frame;
        image_message.header.stamp = ros::Time::now();
        _image_pub.publish(image_message.toImageMsg());

        //std::cout << "Got frame" << std::endl;

        // INSERT BREAK HERE
        // STOP SOUND 
        // sc.stopWave(filename + ".wav");

        if (_stop == true){
        	//std::cout << "Thread terminating before video ends" << std::endl;
        	break;
        } else {
            //std::cout << "Playing video: " << i << std::endl;
        }
        i += 1;
        this_thread::sleep_until(start_time + std::chrono::milliseconds(42));
    }

    // When everything done, release the video capture object
    cap.release();
    
    //cv::Mat blank = cv::Mat::zeros(frame.size(), frame.type());
    //sc.playWave(FOLDER_ROOT + "/blank.wav");

    //cv_bridge::CvImage image_message;
    //image_message.encoding = "bgr8";
    //image_message.image = blank;
    //image_message.header.stamp = ros::Time::now();
    //_image_pub.publish(image_message.toImageMsg());

    // Closes all the frames
    destroyAllWindows();
     
    return 0;
}
