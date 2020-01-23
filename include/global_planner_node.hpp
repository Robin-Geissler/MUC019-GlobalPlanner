#pragma once

#include <tufast_msgs/GoalPoints.h>
#include <tufast_msgs/ControlStatusMessage.h>
#include <tufast_msgs/TabletCommand.h>
#include <tufast_msgs/Lines.h>
#include <tufast_msgs/PerceptionPoints.h>
#include <tufast_msgs/StatusMessage.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_listener.h>

#include "global_planner.hpp"

namespace tufast_planner {

class GlobalPlanner;

class GlobalPlannerNode {
  private:
    ros::Subscriber _subControlStatus;

    tf2_ros::Buffer            _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    ros::Publisher _pubGoalPoses;
    ros::Publisher _pubPath;
    ros::Publisher _pubStatus;

    geometry_msgs::Pose _basePosition;

    tufast_msgs::ControlStatusMessage _old_control_status_messsage;

    GlobalPlanner _gp();

  public:
    GlobalPlannerNode(ros::NodeHandle nh);
    ~GlobalPlannerNode();
};

} // namespace tufast_planner
