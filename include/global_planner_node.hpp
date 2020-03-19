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

namespace tufast_planner {

typedef enum e_MissionType {
    TRACK_DRIVE = 1,
    PARKING     = 2,
    SLALOM      = 3,
    ROLLOUT     = 100
} MissionType;

class GlobalPlannerNode {
  private:
    ros::Subscriber _subBoundingBoxes;
    ros::Subscriber _subOccupancyGridMap;
    ros::Subscriber _subMissionStatus;

    tf2_ros::Buffer            _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    ros::Publisher _pubGoalPoses;
    ros::Publisher _pubPath;

    geometry_msgs::Pose _currentPosition;
    MissionType         _currentMission;

  public:
    GlobalPlannerNode(ros::NodeHandle nh);
    ~GlobalPlannerNode();

    void boundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray& msg);
    void occuGridMapCallback(const nav_msgs::OccupancyGrid& msg);
    void missionStatusCallback(const tufast_msgs::MissionStatus& msg);
};

} // namespace tufast_planner
