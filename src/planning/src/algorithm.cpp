#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridPlanner;

float aStarG(Node2D& start, const Node2D& goal, std::vector<Node2D>& nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization) {
  Node2D* result = Algorithm::aStar(start, goal, nodes2D, width, height, configurationSpace, visualization);
  if (result == nullptr)
    return 1000;
  else
    return result->getG();
}

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        2D A*
//###################################################
Node2D* Algorithm::aStar(Node2D& start,
                         const Node2D& goal,
                         std::vector<Node2D>& nodes2D,
                         int width,
                         int height,
                         CollisionDetection& configurationSpace,
                         Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred;
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc.setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc.isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc.updateG();
            newG = nSucc.getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc.updateH(goal);
              // put successor on open list
              nSucc.open();
              nodes2D[iSucc] = nSucc;
              O.push(&nodes2D[iSucc]);
            }
          }
        }
      }
    }
  }

  // return large number to guide search away
  return nullptr;
}

void Algorithm::tracePath2D(const Node2D* node, int i, std::vector<Node2D>& path) {
  if (node == nullptr) {
    return;
  }

  i++;
  path.push_back(*node);
  tracePath2D(node->getPred(), i, path);
}
