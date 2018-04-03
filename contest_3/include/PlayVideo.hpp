/*
 * PlayVideo.hpp
 *
 *  Created on: Apr 3, 2018
 *      Author: kevin
 */

#ifndef GOOFY_MIE443_CONTEST_3_INCLUDE_PLAYVIDEO_HPP_
#define GOOFY_MIE443_CONTEST_3_INCLUDE_PLAYVIDEO_HPP_

#include<thread>
#include<atomic>
#include"opencv2/opencv.hpp"
#include<iostream>
#include"sound_play/sound_play.h"
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>

class VideoPlayer{
public:
	VideoPlayer(): _stop(false){
		_image_pub = _nh.advertise<sensor_msgs::Image>("/emotion", 10);
	}

	~VideoPlayer();
	void play(std::string filename);
	void stop();

private:
	int PlayVideo(std::string filename);

	ros::NodeHandle _nh;
	ros::Publisher _image_pub;
	std::thread video_player;
	std::atomic<bool> _stop;
};


#endif /* GOOFY_MIE443_CONTEST_3_INCLUDE_PLAYVIDEO_HPP_ */
