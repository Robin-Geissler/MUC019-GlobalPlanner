#include <ros/ros.h>
#include "global_planner_node.hpp"
#include "global_planner.hpp"

using namespace tufast_planner;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "global_planner_node");
  ros::NodeHandle nodeHandle("~");

  GlobalPlannerNode node(nodeHandle);

  ros::spin();
}

GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle nh) : _tfListener{_tfBuffer} {
    
    // -----------------------------------------
    // subscribers
    // ----------------------------------------

    _subControlStatus = nh.subscribe<tufast_msgs::ControlStatusMessage>(
      "/rosbridge/control_status", 1,
      [this](const tufast_msgs::ControlStatusMessageConstPtr&
                 msg) {
        ROS_INFO("Received a control status message");
      });


    // -----------------------------------------
    // transform listener
    // ----------------------------------------

    geometry_msgs::TransformStamped t;
    try {
        t = _tfBuffer.lookupTransform("/map", "/base", ros::Time(0));
        _basePosition.position.x = t.transform.translation.x;
        _basePosition.position.y = t.transform.translation.y;
        _basePosition.position.z = t.transform.translation.z;
        
         _basePosition.orientation = t.transform.rotation;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // -----------------------------------------
    // publishers
    // ----------------------------------------

    _pubGoalPoses = nh.advertise<tufast_msgs::GoalPoints>("goal_poses", 1);
    _pubStatus = nh.advertise<tufast_msgs::StatusMessage>("status", 1);
    _pubPath = nh.advertise<nav_msgs::Path>("path", 1);
}

GlobalPlannerNode::~GlobalPlannerNode() {}
