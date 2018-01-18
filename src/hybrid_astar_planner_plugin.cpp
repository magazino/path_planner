#include <pluginlib/class_list_macros.h>
#include <hybrid_astar_planner_plugin.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(HybridAStar::HybridAStarPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace HybridAStar {

  HybridAStarPlanner::HybridAStarPlanner () : PlannerBase(), costmap(){

  }

  HybridAStarPlanner::HybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
  }

  void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ros::NodeHandle private_nh = ros::NodeHandle("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    nav_msgs::OccupancyGrid occupancy_grid;
    costmap = costmap_ros;
    costmap_ros->getCostmap()->getMutex()->lock();
    costmapToOccupancyGrid(costmap_ros, occupancy_grid);
    PlannerBase::setMap(occupancy_grid);
    costmap_ros->getCostmap()->getMutex()->unlock();
  }

  bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    geometry_msgs::PoseWithCovarianceStamped::Ptr start_covariance(new geometry_msgs::PoseWithCovarianceStamped);
    start_covariance->pose.pose =  start.pose;
    start_covariance->header = start.header;
    PlannerBase::setStart(geometry_msgs::PoseWithCovarianceStamped::ConstPtr(start_covariance));

    geometry_msgs::PoseStamped::Ptr goal_travel(new geometry_msgs::PoseStamped);
    *goal_travel = goal;
    PlannerBase::setGoal(geometry_msgs::PoseStamped::ConstPtr(goal_travel));

    static int kk = 0;
    static ros::WallDuration t;
    ros::WallTime t0 = ros::WallTime::now();

    t += ros::WallTime::now() - t0;
    kk++;
    PlannerBase::plan();
    if (kk%2 == 0){
      ROS_INFO("computing path took: %f", t.toSec());
      t = ros::WallDuration();
    }
//    plan.clear();
    //plan = path.getPath().poses;
    if (path.getPath().poses.size() > 0) {
      std::vector<geometry_msgs::PoseStamped> poses = path.getPath().poses;
      plan = poses;
      //    std::copy(poses.begin(), poses.end(), plan.begin());
      //    plan.assign(path.getPath().poses.begin(),path.getPath().poses.end());
      ROS_INFO_STREAM("Found plan with "<<plan.size()<<" nodes");
      ROS_INFO_STREAM("The frame_id used for navigation is: "<<path.getPath().header.frame_id);
      ROS_INFO_STREAM("Path points have header frame: "<<path.getPath().poses.front().header.frame_id);

      // create and publish a path message to visualize on RViz
      nav_msgs::Path gui_path;
      gui_path.header.frame_id = "map";
      //    gui_path.header.stamp = path.getPath().header.stamp;
      gui_path.poses = poses;
      //    std::copy(poses.begin(), poses.end(), gui_path.poses.begin());
      //    gui_path.poses.assign(path.getPath().poses.begin(),path.getPath().poses.end());
      plan_pub_.publish(gui_path);
      ROS_INFO_STREAM("Publishing plan with "<<gui_path.poses.size()<<" nodes");
      ROS_INFO_STREAM("FIRST POINT: "<<gui_path.poses.front());
    }
    else {
      ROS_ERROR_STREAM("[hybrid_astar_planner] generated path has 0 nodes!");
    }
    return true;
  }

  void HybridAStarPlanner::costmapToOccupancyGrid(costmap_2d::Costmap2DROS *costmap, nav_msgs::OccupancyGrid& occupancy_grid)
  {
    double resolution = costmap->getCostmap()->getResolution();

    occupancy_grid.header.frame_id = costmap->getGlobalFrameID();
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.info.resolution = resolution;

    occupancy_grid.info.width = costmap->getCostmap()->getSizeInCellsX();
    occupancy_grid.info.height = costmap->getCostmap()->getSizeInCellsY();

    double wx, wy;
    costmap->getCostmap()->mapToWorld(0, 0, wx, wy);
    occupancy_grid.info.origin.position.x = wx - resolution / 2;
    occupancy_grid.info.origin.position.y = wy - resolution / 2;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;

    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);

    unsigned char* data = costmap->getCostmap()->getCharMap();
    for (unsigned int i = 0; i < occupancy_grid.data.size(); i++)
    {
      // no typedefs for occupancy grids, just set to 100
      // skip inscribed, inflated and all other intermidiate obstacles, just binary
      if (data[i] == costmap_2d::LETHAL_OBSTACLE)
        occupancy_grid.data[i] = 100;
      else
        occupancy_grid.data[i] = 0;
    }
  }
};
