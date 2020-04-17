#include <ros/ros.h>
#include "global_planner_node.hpp"

using namespace tufast_planner;

int main(int argc, char *argv[]) {




  ros::init(argc, argv, "global_planner_node");
  ros::NodeHandle nodeHandle("~");

  GlobalPlannerNode node(nodeHandle, ros::Rate(100));

  // -----------------------------------------
  // run GlobalPlannerNode
  // ----------------------------------------

  node.run();

}


GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle nh, ros::Rate loop_rate)
        : _tfListener{_tfBuffer}, loop_rate(loop_rate) {
    
    // -----------------------------------------
    // subscribers
    // ----------------------------------------
    _subBoundingBoxes = nh.subscribe("bounding_boxes", 1000, &GlobalPlannerNode::boundingBoxCallback, this);
    _subOccupancyGridMap = nh.subscribe("costmap", 1000, &GlobalPlannerNode::occuGridMapCallback, this);
    _subMissionStatus = nh.subscribe("mission_status", 1000, &GlobalPlannerNode::missionStatusCallback, this);
    
    // -----------------------------------------
    // transform listener
    // ----------------------------------------

    geometry_msgs::TransformStamped t;
    try {
        t = _tfBuffer.lookupTransform("map", "base", ros::Time(0));
        _currentPosition.position.x = t.transform.translation.x;
        _currentPosition.position.y = t.transform.translation.y;
        _currentPosition.position.z = t.transform.translation.z;
        
        _currentPosition.orientation = t.transform.rotation;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // -----------------------------------------
    // publishers
    // ----------------------------------------

    _pubGoalPoses = nh.advertise<tufast_msgs::GoalPoints>("goal_points", 1);
    _pubPath = nh.advertise<nav_msgs::Path>("path", 1);

}


GlobalPlannerNode::~GlobalPlannerNode() {}

void GlobalPlannerNode::boundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray& msg) {
    ROS_INFO("Received a bounding box");
    // TODO
}

void GlobalPlannerNode::occuGridMapCallback(const nav_msgs::OccupancyGrid& msg) {
    ROS_INFO("Received a map");
    // TODO
}

void GlobalPlannerNode::missionStatusCallback(const tufast_msgs::MissionStatus& msg) {
    ROS_INFO("Received a mission status");
    _currentMission = (MissionType) msg.selectedMission;
}

void GlobalPlannerNode::run() {
    // TODO
    while(ros::ok()) {
        // -----------------------------------------
        // sensor fusion
        // ----------------------------------------


        // -----------------------------------------
        // challenge response
        // ----------------------------------------


        ros::spinOnce();
        this->loop_rate.sleep();
    }

}

void GlobalPlannerNode::addGrids(nav_msgs::OccupancyGrid *grid1, nav_msgs::OccupancyGrid grid2) {
    // check height and width
    if(grid1->info.height != grid2.info.height || grid1->info.width != grid2.info.width){
        throw std::range_error("In GlobalPlannerNode::addGrids() grid width and height do not match");
    }

    for(int i = 0; i < grid1->data.size(); i++){
        grid1->data.data()[i] += grid2.data.data()[i];
    }
}
