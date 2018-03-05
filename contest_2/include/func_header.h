#ifndef FUNC_HEADER_H
#define FUNC_HEADER_H

#include <nav_header.h>
#include <imageTransporter.hpp>

using namespace std;

bool init(vector<vector<float> >& coord, std::vector<cv::Mat>& imgs_track);

int findPic(imageTransporter imgTransport, vector<cv::Mat> imgs_track);

class Cereal {
public:
	Cereal (int, int);
	int coord;
	int logo;
};

class Status {
public:
	Status (int, bool);
	int coord;
	bool success;
};

bool getNextCoord(int* coordIndex, int count, vector<Status> mission);


#endif
