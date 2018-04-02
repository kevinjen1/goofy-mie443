
#include "opencv2/opencv.hpp"
#include <iostream>
 
using namespace std;
using namespace cv;
 
int PlayVideo(string filename)
{
    // Create a VideoCapture object and open the input file
    VideoCapture cap(filename + ".mp4"); 
    sound_play::SoundClient sc; 

    // Check if camera opened successfully
    if(!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    // Play Sound
    sc.playWave(filename + ".wav");
    
    // Play Video
    while(1){

        Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
          break;

        // Display the resulting frame
        imshow( "Frame", frame );

        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
          break;

        // INSERT BREAK HERE
        // STOP SOUND 
        // sc.stopWave(filename + ".wav");
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();
     
    return 0;
}