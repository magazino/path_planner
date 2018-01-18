#include "planner.h"
// profiling
#ifdef PROFILING_ENABLED
  #include "gperftools/profiler.h"
#endif

using namespace HybridAStar;

//###################################################
//                                        CONSTRUCTOR
//###################################################
PlannerBase::PlannerBase() : grid(new nav_msgs::OccupancyGrid)
//,nodes2D()
//,nodes3D()
{
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
//void PlannerBase::initializeLookups() {
//  if (Constants::dubinsLookup) {
//    Lookup::dubinsLookup(dubinsLookup);
//  }

//  ROS_INFO_STREAM("Attemping to generate collision lookup table");
//  Lookup::collisionLookup(collisionLookup);
//}

//###################################################
//                                                MAP
//###################################################
void PlannerBase::setMap(const nav_msgs::OccupancyGrid& map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  // we have to copy the static costmap once during initialization
  *grid = map;
  //update the configuration space with the current map
  // use grid as the map pointer WILL be deleted
  configurationSpace.updateGrid(grid);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map.info.height;
  int width = map.info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map.data[y * width + x] ? true : false;
    }
  }

  ros::WallDuration t;
  ros::WallTime t0 = ros::WallTime::now();

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize("/home/denisov/Documents/VoronoiTest/test.pgm");
  t = ros::WallTime::now() - t0;
  ROS_WARN("computing vornoi took: %f", t.toSec());
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void PlannerBase::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void PlannerBase::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void PlannerBase::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

#ifdef PROFILING_ENABLED
    ProfilerStart("path_planner.log");
#endif

    ROS_INFO_STREAM("Beginning planning");
    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    // use map container to be able to use large maps
    std::unordered_map<int,Node3D> nodes3D(length);
    std::unordered_map<int,Node2D> nodes2D(width*height);
    nodes3D.clear();
    nodes2D.clear();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
//    ROS_DEBUG_STREAM("Retrieved goal pose: "<<x<<","<<y<<" (in cells)");
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
//    ROS_DEBUG_STREAM("Retrieved start pose: "<<x<<","<<y<<" (in cells)");
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    if (!nSolution) {
      ROS_ERROR_STREAM("Empty solution returned");
      return;
    }

    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
//    path.updatePath(smoother.getPath());
    path.generatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    // overwrite with smoothed path
    path.clear();
    path.generatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
//    visualization.publishNode3DCosts(nodes3D, width, height, depth);
//    visualization.publishNode2DCosts(nodes2D, width, height);

#ifdef PROFILING
    ProfilerStop();
#endif

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}

//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() : PlannerBase() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

void Planner::plan() {
  PlannerBase::plan();

  if (validStart && validGoal) {
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();

    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;

//    visualization.publishNode3DCosts(PlannerBase::nodes3D, width, height, depth);
//    visualization.publishNode2DCosts(PlannerBase::nodes2D, width, height);
  }
}

void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  PlannerBase::setStart(initial);
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    // publish start for RViz
    pubStart.publish(startN);
  }
}
