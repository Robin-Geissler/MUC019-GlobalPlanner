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
        : _tfListener{_tfBuffer}, _loopRate(loop_rate) {
    
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
        t = _tfBuffer.lookupTransform("map", "base", ros::Time::now(), ros::Duration(10.0));
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
    _pubControlStatus = nh.advertise<tufast_msgs::ControlStatusMessage>("local_planner_data",1);

    // -----------------------------------------
    // rayTracer
    // ----------------------------------------

    _rayTracer = RayTracer(RAYTRACER_RAYNUMBER,STD_OCCU_GRID_HEIGHT,STD_OCCU_GRID_WIDTH);

    // -----------------------------------------
    // boundingBoxMapper
    // ----------------------------------------

    _boundingBoxMapper = initEmptyGrid();

    // -----------------------------------------
    // output Grid
    // ----------------------------------------

    _outputGrid = initEmptyGrid();

    // -----------------------------------------
    // Goal Point
    // ----------------------------------------
    _goalPointX = 0;
    _goalPointY = 0;
}


GlobalPlannerNode::~GlobalPlannerNode() {}

void GlobalPlannerNode::boundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray& msg) {
    ROS_INFO("Received a bounding box");


    if(msg.boxes.size() < 2){
        // no two cones detected
        // set all points to 0
        setGridPoint(&_boundingBoxMapper);
    }else{
        // minimum two cones detected -> take the first two
        Coordinate centerPoint((msg.boxes[0].pose.position.x - msg.boxes[1].pose.position.x)/2,(msg.boxes[0].pose.position.y - msg.boxes[1].pose.position.y)/2);
        setGridPoint(&_boundingBoxMapper,centerPoint.getX(),centerPoint.getY(),100);
    }



}

void GlobalPlannerNode::occuGridMapCallback(const nav_msgs::OccupancyGrid& msg) {
    ROS_INFO("Received a map");
    // check if grid width and height are correct
    if(msg.info.width != STD_OCCU_GRID_WIDTH){
        throw std::range_error("GlobalPlanner::occuGridMapCallback detected that STD_OCCU_GRID_WIDTH is not compatible with incoming grids" );
    }
    if(msg.info.height != STD_OCCU_GRID_HEIGHT){
        throw std::range_error("GlobalPlanner::occuGridMapCallback detected that STD_OCCU_GRID_HEIGHT is not compatible with incoming gridsy" );
    }

    _rayTracer.setInputGrid(msg);
}

void GlobalPlannerNode::missionStatusCallback(const tufast_msgs::MissionStatus& msg) {
    ROS_INFO("Received a mission status");
    _currentMission = (MissionType) msg.selectedMission;
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


Coordinate GlobalPlannerNode::getGoalPoint(nav_msgs::OccupancyGrid outputGrid){
    int outputVal = 0;
    Coordinate outputGridPoint(0,0);
    for(int i = 0; i < outputGrid.data.size(); i++){
        if(outputGrid.data[i] > outputVal){
            outputVal = outputGrid.data[i];
            outputGridPoint.setX(i % outputGrid.info.width);
            outputGridPoint.setY(i / outputGrid.info.width);
        }
    }
    return outputGridPoint;
}

nav_msgs::OccupancyGrid GlobalPlannerNode::initEmptyGrid() {
    nav_msgs::OccupancyGrid grid;
    grid.info.width = STD_OCCU_GRID_WIDTH;
    grid.info.height = STD_OCCU_GRID_HEIGHT;
    // init outputGrid with 0
    for(int i = 0; i < STD_OCCU_GRID_WIDTH *  STD_OCCU_GRID_HEIGHT; i++){
        grid.data.push_back(0);
    }

    return grid;
}

void GlobalPlannerNode::setGridPoint(nav_msgs::OccupancyGrid *grid, int xPos, int yPos, float val) {
    int index = yPos * grid->info.width + xPos;
    for(int i = 0; i < grid->info.width * grid->info.height; i++){
        if(i == index){
            grid->data[i] = val;
        }else{
            grid->data[i] = 0;
        }
    }
}

void GlobalPlannerNode::run() {
    // init Messages
    tufast_msgs::GoalPoints goalPointMsg;
    tufast_msgs::ControlStatusMessage controlStatusMsg;
    nav_msgs::Path pathMsg;

    // init help Coordinates
    Coordinate newGoalPoint(0, 0);
    float oldGoalPointX;
    float oldGoalPointY;

    while(ros::ok()) {
        // -----------------------------------------
        // get sensor data
        // ----------------------------------------

        // set Global Planner outputGrid to 0
        for(int i = 0; i < _outputGrid.data.size(); i++){
            _outputGrid.data.data()[i] = 0;
        }

        // set rayTracer outputGrid
        _rayTracer.setOutputGrid();

        // the _boundingBoxMapper is already set

        // TODO init more dataGrids here

        // -----------------------------------------
        // sensor fusion
        // ----------------------------------------

        switch (_currentMission) {
            case TRACK_DRIVE:
                // add RayTracer data
                addGrids(&_outputGrid, _rayTracer.getOutputGrid());
                break;
            case PARKING:
                // TODO Parking is still to be implemented
                break;
            case SLALOM:
                // add RayTracer data
                addGrids(&_outputGrid, _rayTracer.getOutputGrid());
                // add BoundingBoxMapper data
                addGrids(&_outputGrid,_boundingBoxMapper);
                break;
            case ROLLOUT:
                // just stand still
                break;
        }

        // -----------------------------------------
        // generate path data

        // ----------------------------------------

        // get GoalPoints
        // TODO publish the Goalpoints here
        newGoalPoint = getGoalPoint(_outputGrid);
        oldGoalPointX = _goalPointX;
        oldGoalPointY = _goalPointY;

        // calculate distance from Car Origin in m
        // probably switch X and Y Corrdinate here
        _goalPointX = (float)newGoalPoint.getX() * STD_OCCU_GRID_RESOLUTION;
        _goalPointY = ((float)newGoalPoint.getY() - (((float)STD_OCCU_GRID_WIDTH - 1)/2)) * (float)STD_OCCU_GRID_RESOLUTION;
        //ROS_INFO("%f \n %f",_goalPointX, _goalPointY);

        // set output GoalPoints msg
        if(goalPointMsg.x.size() > 0 && goalPointMsg.y.size() > 0){
        goalPointMsg.x.pop_back();
        goalPointMsg.y.pop_back();
        }
        goalPointMsg.x.push_back(_goalPointX);
        goalPointMsg.y.push_back(_goalPointY);

        // set output Path msg
        // TODO generate Path msg

        // set output ControlStatus msg
        controlStatusMsg.sx = _goalPointX;
        controlStatusMsg.sy = _goalPointY;

        // for big position changes set replan options
        if(abs(_goalPointX - oldGoalPointX) > 0.5 ){
            controlStatusMsg.replanLat = 1;
        } else{
            controlStatusMsg.replanLat = 0;
        }

        if(abs(_goalPointY - oldGoalPointY) > 0.5){
            controlStatusMsg.replanLong = 1;
        } else{
            controlStatusMsg.replanLong = 0;
        }

        // -----------------------------------------
        // publish path data
        // ----------------------------------------

        // publish msgs
        _pubGoalPoses.publish(goalPointMsg);
        // TODO add Path msg
        _pubControlStatus.publish(controlStatusMsg);

        ros::spinOnce();
        this->_loopRate.sleep();
    }

}






