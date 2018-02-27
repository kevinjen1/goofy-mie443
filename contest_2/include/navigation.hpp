namespace goofy{
namespace navigation{

struct Pose{
  double x;
  double y;
  double theta;
}

std::vector<Pose> readXML(std::string file_location); //note that the start position will not be in this list

Pose getNextPoint(geometry_msgs::Pose cur_pos, std::vector<Pose> dest_list);

}
}
