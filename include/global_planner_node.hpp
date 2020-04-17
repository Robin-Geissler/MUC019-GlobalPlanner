#pragma once

#include <tufast_msgs/GoalPoints.h>
#include <tufast_msgs/ControlStatusMessage.h>
#include <tufast_msgs/TabletCommand.h>
#include <tufast_msgs/Lines.h>
#include <tufast_msgs/PerceptionPoints.h>
#include <tufast_msgs/StatusMessage.h>
#include <tufast_msgs/MissionStatus.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <tf2_ros/transform_listener.h>

#include "rayTracer.hpp"

#define STD_OCCU_GRID_WIDTH 150  //Width of the expected Occu Grids
#define STD_OCCU_GRID_HEIGHT 150 //Height of the expected Occu Grids

namespace tufast_planner {

typedef enum e_MissionType {
    TRACK_DRIVE = 1,
    PARKING     = 2,
    SLALOM      = 3,
    ROLLOUT     = 100
} MissionType;

class GlobalPlannerNode {
  private:
    ros::Subscriber _subBoundingBoxes;              // jsk_recognition_msgs::BoundingBoxArray
    ros::Subscriber _subOccupancyGridMap;           // nav_msgs::OccupancyGrid
    ros::Subscriber _subMissionStatus;              // tufast_msgs::MissionStatus

    tf2_ros::Buffer            _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    ros::Publisher _pubGoalPoses;
    ros::Publisher _pubPath;

    geometry_msgs::Pose _currentPosition;
    MissionType         _currentMission;

    ros::Rate loop_rate;

    RayTracer rayTracer;

    nav_msgs::OccupancyGrid outputGrid;

    void boundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray& msg);
    void occuGridMapCallback(const nav_msgs::OccupancyGrid& msg);
    void missionStatusCallback(const tufast_msgs::MissionStatus& msg);

    /**
     * @brief adds the grids, result will be saved in grid1
     * @param grid1 pointer to first summand and output Grid
     * @param grid2 second summand
     */
    void addGrids(nav_msgs::OccupancyGrid *grid1,nav_msgs::OccupancyGrid grid2);

  public:
    GlobalPlannerNode(ros::NodeHandle nh, ros::Rate loop_rate);
    ~GlobalPlannerNode();

    void run();
};

} // namespace tufast_planner
