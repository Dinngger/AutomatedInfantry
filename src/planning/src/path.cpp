#include "path.h"

using namespace HybridPlanner;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  path.poses.clear();
  publishPath();
}

//###################################################
//                                         TRACE PATH
//###################################################

void Path::updatePath2D(std::vector<Node2D> nodePath) {
  path.header.stamp = ros::Time::now();

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment2D(nodePath[i]);
  }

  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment2D(const Node2D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = (0.5 + node.getX()) * Constants::cellSize;
  vertex.pose.position.y = (0.5 + node.getY()) * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}
