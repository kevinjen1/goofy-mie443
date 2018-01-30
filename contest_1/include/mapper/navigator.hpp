//#ifndef GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_
//#define GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_

#include "geometry_msgs/Pose2D.h"
#include "mapper/mapper.hpp"

namespace goofy{
namespace mapper{

geometry_msgs::Pose2D getCoordinate(goofy::mapper::LocalMap grid);
void lookForCoordinates(const LocalMap grid);
ros::Publisher coord_pub;

}}

//#endif /* GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_ */
