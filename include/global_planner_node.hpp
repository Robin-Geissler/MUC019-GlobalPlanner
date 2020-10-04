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

#define STD_OCCU_GRID_WIDTH 155  //Width of the expected Occu Grids
#define STD_OCCU_GRID_HEIGHT 155 //Height of the expected Occu Grids
#define STD_OCCU_GRID_RESOLUTION 0.09 // Resolution of the expected Occu Grid

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
    ros::Publisher _pubControlStatus;

    geometry_msgs::Pose _currentPosition;
    MissionType         _currentMission;

    ros::Rate _loopRate;

    RayTracer _rayTracer;
    nav_msgs::OccupancyGrid _boundingBoxMapper;

    nav_msgs::OccupancyGrid _outputGrid;

    /// represents the physical goal Point in m
    float _goalPointX;
    float _goalPointY;

    void boundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray& msg);
    void occuGridMapCallback(const nav_msgs::OccupancyGrid& msg);
    void missionStatusCallback(const tufast_msgs::MissionStatus& msg);

    /**
     * @brief adds the grids, result will be saved in grid1
     * @param grid1 pointer to first summand and output Grid
     * @param grid2 second summand
     */
    void addGrids(nav_msgs::OccupancyGrid *grid1,nav_msgs::OccupancyGrid grid2);

    /**
     * @brief calculates the highest valued point of the output grid
     * @param outputGrid the grid that is scanned
     * @return the highest valued point
     */
    Coordinate getGoalPoint(nav_msgs::OccupancyGrid outputGrid);

    /**
     * @brief generates an empty OccupancyGrid
     * @param
     * @return the empty grid
     */
    static nav_msgs::OccupancyGrid initEmptyGrid();

    /**
     * @brief sets one point in an OccupancyGrid
     * @param grid a pointer to the grid
     * @param xPos the x position of the point to set
     * @param yPos the y position of the point to set
     * @param val the value of the point
     * @details the output grid will have the point set accordingly,
     * all other points in the grid are set to 0
     */
    static void setGridPoint(nav_msgs::OccupancyGrid *grid, int xPos = 0, int yPos = 0, float val = 0);

  public:
    GlobalPlannerNode(ros::NodeHandle nh, ros::Rate loop_rate);
    ~GlobalPlannerNode();

    void run();


};

} // namespace tufast_planner
