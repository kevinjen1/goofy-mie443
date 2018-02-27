#include <func_header.h>
#include <ros/package.h>

bool init(vector<vector<float> >& coord, std::vector<cv::Mat>& imgs_track){
	string packpath = ros::package::getPath("mie443_contest2");

	//*****************************  XML FilePath  *********************************
	string filepath = packpath + "/src/input.xml";
	//******************************************************************************

	
	cv::FileNode xNode, yNode, thetaNode, imageNode;
	cv::FileNodeIterator xIt, yIt, thetaIt, imageIt;
	cv::FileNodeIterator xEnd, yEnd, thetaEnd, imageEnd;

	//open XML
	cv::FileStorage fs(filepath, cv::FileStorage::READ);

	if(fs.isOpened()){
		cv::FileNode node;
		cv::FileNodeIterator it, end;

		//load coordinates
		vector<float> coordVec;
		string coords[5] = {"coordinate1", "coordinate2", "coordinate3", "coordinate4", "coordinate5"};

		for(int i = 0; i < 5; ++i){
			node = fs[coords[i]];
			if(node.type() != cv::FileNode::SEQ){
				cout << "Data in " << coords[i] << " is improperly formatted - check input.xml" << endl; 
				continue;
			}

			it = node.begin();
			end = node.end();

			coordVec = vector<float>();

			for(int j = 0; it != end; ++it, ++j){
				if(j >= 3){
					cout << "Data in " << coords[i] << " is improperly formatted - check input.xml" << endl; 
					coordVec.clear();
					break;
				}
				coordVec.push_back((float)*it);
			}
			if(coordVec.size() == 3) coord.push_back(coordVec);
		}

		if(coord.size() == 0){
			cout << "coordinate data is improperly formatted - check input.xml" << endl;
			return false;
		}

		//load images
		node = fs["images"];

		if(!(node.type() == cv::FileNode::SEQ || node.type() == cv::FileNode::STRING)){ 
			cout << "image data is improperly formatted - check input.xml" << endl;
			return false;
		}

		it = node.begin();
		end = node.end();

		string imagepath;
		for(; it != end; ++it){
			imagepath = packpath + (string)*it;
			imgs_track.push_back(cv::imread(imagepath, CV_LOAD_IMAGE_GRAYSCALE));
		}		
	}else{
		cout << "Could not open XML - check FilePath in ../src/accessory/init.cpp" << endl;
		return false;
	}
	return true;
}
