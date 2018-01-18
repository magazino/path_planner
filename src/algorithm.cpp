#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
void updateH(Node3D& start, const Node3D& goal, std::unordered_map<int,Node2D>& nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               std::unordered_map<int,Node3D>& nodes3D,
                               std::unordered_map<int,Node2D>& nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // VISUALIZATION DELAY
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  // update h value
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  // float max = 0.f;

  // CHECK FOR TRACE BREAKS
  const Node3D* traced_node;
  bool loop_found = false;
  std::set<int> trace;

  // continue until O empty
  while (!O.empty()) {

    //    // DEBUG
    //    Node3D* pre = nullptr;
    //    Node3D* succ = nullptr;

    //    std::cout << "\t--->>>" << std::endl;

    //    for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it) {
    //      succ = (*it);
    //      std::cout << "VAL"
    //                << " | C:" << succ->getC()
    //                << " | x:" << succ->getX()
    //                << " | y:" << succ->getY()
    //                << " | t:" << helper::toDeg(succ->getT())
    //                << " | i:" << succ->getIdx()
    //                << " | O:" << succ->isOpen()
    //                << " | pred:" << succ->getPred()
    //                << std::endl;

    //      if (pre != nullptr) {

    //        if (pre->getC() > succ->getC()) {
    //          std::cout << "PRE"
    //                    << " | C:" << pre->getC()
    //                    << " | x:" << pre->getX()
    //                    << " | y:" << pre->getY()
    //                    << " | t:" << helper::toDeg(pre->getT())
    //                    << " | i:" << pre->getIdx()
    //                    << " | O:" << pre->isOpen()
    //                    << " | pred:" << pre->getPred()
    //                    << std::endl;
    //          std::cout << "SCC"
    //                    << " | C:" << succ->getC()
    //                    << " | x:" << succ->getX()
    //                    << " | y:" << succ->getY()
    //                    << " | t:" << helper::toDeg(succ->getT())
    //                    << " | i:" << succ->getIdx()
    //                    << " | O:" << succ->isOpen()
    //                    << " | pred:" << succ->getPred()
    //                    << std::endl;

    //          if (pre->getC() - succ->getC() > max) {
    //            max = pre->getC() - succ->getC();
    //          }
    //        }
    //      }

    //      pre = succ;
    //    }

    // pop node with lowest cost from priority queue
    nPred = O.top();
//    if (nPred->getPred()) {
//      ROS_DEBUG_STREAM("[3D Astar] nPred idx: "<<nPred->getIdx()<<" predecessor: "<<nPred->getPred()->getIdx()<<" with coordinates: "<<nPred->getX()<<" , "<<nPred->getY()
//                      <<" G: "<<nPred->getG()<<" H: "<<nPred->getH()<<" total(C): "<<nPred->getC()
//                      <<" distance to goal: "<<std::sqrt(std::pow(goal.getX()-nPred->getX(),2) + std::pow(goal.getY()-nPred->getY(),2) ));
//      if (nPred->getIdx() == nPred->getPred()->getIdx()) {
//        ROS_DEBUG_STREAM("[3D Astar] SAME PRED AND AFTER IDX(loop)! "
//                         "coords of current: "<<nPred->getX()<<" , "<<nPred->getY()<<" previous: "<<nPred->getPred()->getX()<<" , "<<nPred->getPred()->getY());
//      }
//    }
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;

    // RViz visualization
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D.find( iPred ) != nodes3D.end() && nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D.find(iPred) != nodes3D.end() && nodes3D[iPred].isOpen() ) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal || iterations > Constants::iterations) {
        // DEBUG
        if (iterations > Constants::iterations)
          ROS_WARN_STREAM("Exceeded maximum number of iterations, returning path found so far");
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
            ROS_INFO("Dubins shot succeded, arrived at goal");
            return nSucc;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height)) {

            if (configurationSpace.isTraversable(nSucc)) {

              // ensure successor is not on closed list or it has the same index as the predecessor
              // if it has not been explored yet, it is not in the map
               if ( (nodes3D.find(iSucc) != nodes3D.end() && !nodes3D[iSucc].isClosed()) || nodes3D.find(iSucc) == nodes3D.end() || iPred == iSucc) {

                // calculate new G value
                nSucc->updateG();
                newG = nSucc->getG();

                if (nodes3D.find(iSucc) == nodes3D.end()) {
                  nodes3D[iSucc] = Node3D();
                }

                // trace the path to prevent loops from occuring (will fail to trace and smooth path otherwise)
                traced_node = nPred->getPred();
                while (traced_node) {
                  trace.insert(traced_node->getIdx());
                  traced_node = traced_node->getPred();
                }
                if (trace.find(iSucc) != trace.end()) {
                  ROS_WARN_STREAM("[3D Astar] LOOP FOUND for "<<nSucc->getX()<<" , "<<nSucc->getY());
                  loop_found = true;
                }

                // if successor not on open list or found a shorter way to the cell
                if ( nodes3D.find(iSucc) == nodes3D.end() || (nodes3D.find(iSucc) != nodes3D.end() && !nodes3D[iSucc].isOpen()) ||
                     (nodes3D.find(iSucc) != nodes3D.end() && newG < nodes3D[iSucc].getG()) || iPred == iSucc && !loop_found) {

                  // calculate H value
                  updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                  // if the successor is in the same cell but the C value is larger
                  if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                    delete nSucc;
                    continue;
                  }
                  // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                  else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
//                    ROS_DEBUG_STREAM("[3D Astar] Updating index "<<iPred<<" with new, better node at: "<<nSucc->getX()<<" , "<<nSucc->getY()<<
//                                    " and new cost: "<<nSucc->getC()<<" old cost: "<<nPred->getC());
                    nSucc->setPred(nPred->getPred());
                  }

                  // this should never happen since we check the entire path all the time
                  if (nSucc->getPred() == nSucc) {
                    ROS_WARN_STREAM("LOOPING");
                  }

                  // put successor on open list
                  nSucc->open();
                  nodes3D[iSucc] = *nSucc;
                  O.push(&nodes3D[iSucc]);
//                  ROS_DEBUG_STREAM("[3D Astar] Set/updated id: "<<iSucc<<" with node position: "<<nSucc->getX()<<" , "<<nSucc->getY()<<" to score: "<<nSucc->getC()
//                                  <<" and goal distance: "<<std::sqrt(std::pow(goal.getX()-nSucc->getX(),2) + std::pow(goal.getY()-nSucc->getY(),2) ));
                  delete nSucc;
                } else { delete nSucc; }
              } else { delete nSucc; }
            } else { delete nSucc; }
          }
        }

        trace.clear();
      }
    }
  }

  if (O.empty()) {
    ROS_ERROR_STREAM("[3D Astar] Search graph exhausted, no solution found");
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            std::unordered_map<int,Node2D>& nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (std::unordered_map<int, Node2D>::iterator it = nodes2D.begin(); it != nodes2D.end(); ++it)
  {
    it->second.reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

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
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D.find(iPred) != nodes2D.end() && nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    // have to make sure the node is actually not in the map yet
    else if (nodes2D.find(iPred) != nodes2D.end() && nodes2D[iPred].isOpen()) {

      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
//        ROS_DEBUG_STREAM("[2D Astar] Goal found with C: "<<nPred->getC());
        visualization.clear2D();
        return nPred->getG();
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
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) &&
              (nodes2D.find(iSucc)==nodes2D.end() || ( nodes2D.find(iSucc)!=nodes2D.end() && !nodes2D[iSucc].isClosed())) ) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list (cost difference needs to be at least above the step threshold)
            if ( (nodes2D.find(iSucc) != nodes2D.end() && (!nodes2D[iSucc].isOpen() || std::abs(newG - nodes2D[iSucc].getG()) > HybridAStar::Constants::twoD_astar_scaling) )
                 || nodes2D.find(iSucc) == nodes2D.end() ) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (Constants::visualization2D)
    visualization.clear2D();

  // return large number to guide search away
  ROS_ERROR_STREAM("[2D Astar] Path to goal not found");
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D& start, const Node3D& goal, std::unordered_map<int,Node2D>& nodes2D, float* dubinsLookup, int width,
             int height, CollisionDetection& configurationSpace, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / Constants::deltaHeadingRad);
    //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
    //                                + uY *  Constants::headings * Constants::headings
    //                                + h0 * Constants::headings
    //                                + h1];
    //    } else {

    /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, Constants::r, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

//  const int steps = 100;
//  static int kk = 0;
//  static ros::WallDuration t;
//  ros::WallTime t0 = ros::WallTime::now();

  // if reversing is active use a
  if (Constants::reverse && !Constants::dubins) {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

//  t += ros::WallTime::now() - t0;
//  kk++;
//  if (kk%steps == 0){
//    ROS_DEBUG("[3D Astar H Update] OMPL took: %f over %i steps", t.toSec(), steps);
//    t = ros::WallDuration();
//  }


  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
//  static int kktwo = 0;
//  static ros::WallDuration ttwo;
//  ros::WallTime t1 = ros::WallTime::now();

  // need to check whether the given node is in the map at all
  if (Constants::twoD && ( nodes2D.find((int)start.getY() * width + (int)start.getX()) == nodes2D.end()  ||
     (nodes2D.find((int)start.getY() * width + (int)start.getX()) != nodes2D.end() &&
        !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered() ) ) ) {

    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    if (nodes2D.find((int)start.getY() * width + (int)start.getX()) == nodes2D.end())
      nodes2D[(int)start.getY() * width + (int)start.getX()] = Node2D();
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
    if (Constants::visualization2D)
      visualization.clear2D();
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

//  ttwo += ros::WallTime::now() - t1;
//  kktwo++;
//  if (kktwo%steps == 0){
//    ROS_DEBUG("[3D Astar H Update] 2D grid search took: %f over %i steps", ttwo.toSec(), steps);
//    ttwo = ros::WallDuration();
//  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

  }

//  ROS_DEBUG_STREAM("[2D Astar] H: Position: "<<start.getX()<<" , "<<start.getY()<<" heuristic: reedsShepp "<<reedsSheppCost<<" dubins "<<dubinsCost<<" twoD: "<<twoDCost<<
//                  " C: "<<start.getC()<<" distance to goal: "<<std::sqrt(std::pow(goal.getX()-start.getX(),2) + std::pow(goal.getY()-start.getY(),2) ) );

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      ROS_WARN_STREAM("Dubins shot collided, discarding the path");
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}
