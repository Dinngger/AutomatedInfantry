#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"

int main(int argc, char** argv) {
  printf("Hybrid Planner\nA path-finding algorithm on topological and grids map\n");
  ros::init(argc, argv, "hybrid_planner");
  HybridPlanner::Planner hy;
  ros::spin();
  return 0;
}
