/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// headers required by the hybrid_astar planner
#include <planner.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::string;

#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H

namespace HybridAStar {

class HybridAStarPlanner : public nav_core::BaseGlobalPlanner, protected PlannerBase {
public:

 HybridAStarPlanner();
 HybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

 /** overridden classes from interface nav_core::BaseGlobalPlanner **/
 void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
 bool makePlan(const geometry_msgs::PoseStamped& start,
               const geometry_msgs::PoseStamped& goal,
               std::vector<geometry_msgs::PoseStamped>& plan
              );
private:
 void costmapToOccupancyGrid(costmap_2d::Costmap2DROS* costmap, nav_msgs::OccupancyGrid& occupancy_grid);
 costmap_2d::Costmap2DROS* costmap;
 ros::Publisher plan_pub_;
};
};
#endif
