#include "rayTracer.hpp"

#include <ros/ros.h>    // this is just to display warnings, can be removed in the end

/********************************************************
 *  Class Coordinate
 ********************************************************/

Coordinate::Coordinate(int x, int y) : x(x), y(y) {}

int Coordinate::getX() const {
    return x;
}

void Coordinate::setX(int x) {
    Coordinate::x = x;
}

int Coordinate::getY() const {
    return y;
}

void Coordinate::setY(int y) {
    Coordinate::y = y;
}

Coordinate Coordinate::operator+(const Coordinate &coo) {
    return Coordinate(x + coo.x, y + coo.y);
}

Coordinate Coordinate::operator-(const Coordinate &coo) {
    return Coordinate(x - coo.x, y - coo.y);
}

Coordinate Coordinate::operator*(const float &factor) {
    return Coordinate(x * factor, y * factor);
}

Coordinate Coordinate::operator/(const float &divisor) {
    return Coordinate(x / divisor, y / divisor);
}


/********************************************************
 *  Class Vec2
 ********************************************************/

Vec2::Vec2(float x, float y) : x(x), y(y) {}

Vec2::~Vec2() = default;

float Vec2::getX() const {
    return x;
}

void Vec2::setX(float x) {
    Vec2::x = x;
}

float Vec2::getY() const {
    return y;
}

void Vec2::setY(float y) {
    Vec2::y = y;
}

Vec2 Vec2::operator+(const Vec2& vec) {
    return Vec2(x+vec.x,y+vec.y);
}

Vec2 Vec2::operator-(const Vec2& vec) {
    return Vec2(x-vec.x, y-vec.y);
}

Vec2 Vec2::operator*(const float& factor) {
    return Vec2(x*factor, y*factor);
}


/********************************************************
 *  Class Ray
 ********************************************************/

int Ray::getCoodinateIndex(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // calculate index
    int index = (field.getY() * grid.info.width) + field.getX();

    // asked field is out of Bounds
    if(index > grid.data.size()){
        throw std::range_error("field in Ray::getCoordinateIndex() is out of Bounds");
    }

    return index;
}

bool Ray::outOfBounds(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // check bounds
    return field.getX() >= grid.info.width || field.getY() >= grid.info.height;
}

bool Ray::occupied(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // TODO
    std::cerr << "occupied in Class Ray is not yet implemented" << std::endl;
    return false;
}



int Ray::decAbs(float *num) {
    if(*num > 0){
        (*num)--;
        return 1;
    } else{
        (*num)++;
        return -1;
    }
}

Ray::Ray(const Coordinate &start, const Vec2 &dir) : start(start), dir(dir) {
   }

Ray::~Ray() = default;

const Coordinate &Ray::getStart() const {
    return start;
}

void Ray::setStart(const Coordinate &start) {
    Ray::start = start;
}

const Vec2 &Ray::getDir() const {
    return dir;
}

void Ray::setDir(const Vec2 &dir) {
    Ray::dir = dir;
}

float Ray::getLength() {
    float horizontalLength = occuGridFields.back().getX() - start.getX();
    float verticalLength = occuGridFields.back().getY() - start.getY();

    return sqrtf(horizontalLength*horizontalLength + verticalLength*verticalLength);
}

Coordinate Ray::getCenter() {
    return (occuGridFields.back() - start) / 2;
}

void Ray::setOccuGridFields(nav_msgs::OccupancyGrid &inputGrid) {
    int curX = Ray::getStart().getX();  // current x coordinate
    int curY = Ray::getStart().getY();  // current y coordinate
    Coordinate newPoint(curX,curY);     // new Ray Point

    bool xIsMainDir = abs(Ray::getDir().getX()) >= abs(Ray::getDir().getY()); // tells if x is bigger than y
    float NextStepsX;   // tells how many steps to go in x direction before reload
    float NextStepsY;   // tells how many steps to go in y direction before reload

    // init NextSteps
    // only half way for the main direction in first init
    // this will be full way after a refill
    if(xIsMainDir){
        NextStepsX = Ray::getDir().getX() / 2;
        NextStepsY = Ray::getDir().getY();
    }else{
        NextStepsX = Ray::getDir().getX();
        NextStepsY = Ray::getDir().getY() / 2;
    }

    // end the Loop if the Ray Collides with something
    while(!outOfBounds(newPoint,inputGrid) && !occupied(newPoint,inputGrid)){
        // add new Ray Point
        Ray::occuGridFields.push_back(newPoint);

        // find next Ray Point
        if(xIsMainDir){
            // go to next point, adjust NextSteps and cur accordingly
            newPoint = getNextGridPoint(&NextStepsX, &NextStepsY, &curX, &curY);
        } else{
            newPoint = getNextGridPoint(&NextStepsY, &NextStepsX, &curY, &curX);
        }

        // if nextSteps in x and y direction is empty, refill it here
        if(abs(NextStepsX) < 1 && abs(NextStepsY) < 1){
            NextStepsX +=Ray::getDir().getX();
            NextStepsY +=Ray::getDir().getY();
        }
    }
}

Coordinate Ray::getNextGridPoint(float *mainDirNextSteps, float *subDirNextSteps, int *mainDirCoordinate, int *subDirCoordinate) {
    // go in main dir until it's empty
    if(abs(*mainDirNextSteps) >= 1){
        // go 1 step in x dir and dec mainDirNextSteps by 1
        *mainDirCoordinate += decAbs(mainDirNextSteps);
    }
    // go in sub dir
    else{
        // go 1 step in sub dir and dec subDirNextSteps by 1
        *subDirCoordinate += decAbs(subDirNextSteps);
    }
    return {*mainDirCoordinate, *subDirCoordinate};
}






/********************************************************
 *  Class RayTracer
 ********************************************************/
/*
 * Breef:
 * This is the Constructor for the RayTracer.
 * It Constructs numberOfRays Rays.
 * All Rays will start at RAYTRACER_RAY_START_X and RAYTRACER_RAY_START_Y.
 * Direction of the generated Rays:
 * The direction of the Rays will be equally distributed between 0 and 180 degrees.
 * 0 degrees and 180 degrees are not included.
 * The Vector will be scalled, so that the abs value of the coordinate with lower abs value will be 1
 * @param numberofRays number of rays generated by the Ray Tracer
 */
RayTracer::RayTracer(int numberOfRays){

    // exception: numberOfRays is to low
    if(numberOfRays < 1){
        throw std::range_error("numberOfRays in RayTracer::Raytracer() is out of Range");
    }

    // init rays
    std::vector<Ray> rayVec;
    for(int i = 1; i <= numberOfRays; i++){
        /*
         * Create 1 Ray per loop cycle
         */
        // all rays should have the same angle to each other, so coordinates from the unit circle will be used
        float angle = ((float)i/(float)(numberOfRays + 1) * M_PIf32);
        Vec2 dir(cos(angle),sin(angle));
        // scale dir to set lower coordinate to +1 or -1
        // except if one coordinate is 0
        if(abs(dir.getX()) >= 0.0001 && abs(dir.getY() >= 0.0001)) {
            if (abs(dir.getX()) < abs(dir.getY())) {
                dir = dir * (1 / abs(dir.getX()));
            } else {
                dir = dir * (1 / abs(dir.getY()));
            }
        }
        Ray ray(Coordinate(RAYTRACER_RAY_START_X, RAYTRACER_RAY_START_Y),dir);
        rayVec.push_back(ray);
    }
    RayTracer::rays = rayVec;


    // init inputGrid

    // init outputGrid

}

const std::vector<Ray> &RayTracer::getRays() const {
    return rays;
}

Ray RayTracer::getLongestRay() {
    // TODO
    std::cerr << "getLongestRay in Class RayTracer is not yet implemented" << std::endl;
    return Ray(Coordinate(0,0), Vec2(0,0));
}

const nav_msgs::OccupancyGrid &RayTracer::getInputGrid() const {
    return inputGrid;
}

void RayTracer::setInputGrid(const nav_msgs::OccupancyGrid &inputGrid) {
    RayTracer::inputGrid = inputGrid;
}

const nav_msgs::OccupancyGrid &RayTracer::getOutputGrid() const {
    return outputGrid;
}

void RayTracer::setOutputGrid(const nav_msgs::OccupancyGrid &outputGrid) {
    RayTracer::outputGrid = outputGrid;
}



