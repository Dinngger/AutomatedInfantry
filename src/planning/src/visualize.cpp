#include "visualize.h"
#include <limits>
using namespace HybridPlanner;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
void Visualize::clear() {
  poses2D.poses.clear();

  // 2D COSTS
  visualization_msgs::MarkerArray costCubes2D;
  visualization_msgs::Marker costCube2D;
  // CLEAR THE COST HEATMAP
  costCube2D.header.frame_id = "map";
  costCube2D.header.stamp = ros::Time::now();
  costCube2D.id = 0;
  costCube2D.action = 3;
  costCubes2D.markers.push_back(costCube2D);
  pubNodes2DCosts.publish(costCubes2D);
}

//###################################################
//                                    CURRENT 2D NODE
//###################################################
void Visualize::publishNode2DPose(Node2D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
  pose.pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // PUBLISH THE POSE
  pubNode2D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 2D NODES
//###################################################
void Visualize::publishNode2DPoses(Node2D& node) {
  if (node.isDiscovered()) {
    geometry_msgs::Pose pose;
    pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
    pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);

    poses2D.poses.push_back(pose);
    poses2D.header.stamp = ros::Time::now();
    // PUBLISH THE POSEARRAY
    pubNodes2D.publish(poses2D);

  }
}

//###################################################
//                                    COST HEATMAP 2D
//###################################################
void Visualize::publishNode2DCosts(const std::vector<Node2D>& nodes, int width, int height) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // ________________________________
  // DETERMINE THE MAX AND MIN VALUES
  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    // set the minimum for the cell
    if (nodes[i].isDiscovered()) {
      values[i] = nodes[i].getG();

      // set a new minimum
      if (values[i] > 0 && values[i] < min) { min = values[i]; }

      // set a new maximum
      if (values[i] > 0 && values[i] > max) { max = values[i]; }
    }
  }

  // _______________
  // PAINT THE CUBES
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (nodes[i].isDiscovered()) {
      count++;

      // delete all previous markers
      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }


      costCube.header.frame_id = "map";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "2D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 2D nodes expanded " << std::endl;
  }

  // PUBLISH THE COSTCUBES
  pubNodes2DCosts.publish(costCubes);
}
