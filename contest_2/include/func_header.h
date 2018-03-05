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

Cereal::Cereal (int one, int two) {
	coord = one;
	logo = two;
}

class Status {
public:
	Status (int, bool);
	int coord;
	bool success;
};

Status::Status(int one, bool two) {
	coord = one;
	success = two;
}

#endif
