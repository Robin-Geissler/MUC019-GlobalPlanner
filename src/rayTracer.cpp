#include "rayTracer.hpp"

#include <ros/ros.h>    // this is just to display warnings, can be removed in the end

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

/********************************************************
 *  Class Ray
 ********************************************************/

Ray::Ray(const Vec2 &start, const Vec2 &dir) : start(start), dir(dir) {
}

Ray::~Ray() = default;

const Vec2 &Ray::getStart() const {
    return start;
}

void Ray::setStart(const Vec2 &start) {
    Ray::start = start;
}

const Vec2 &Ray::getDir() const {
    return dir;
}

void Ray::setDir(const Vec2 &dir) {
    Ray::dir = dir;
}

float Ray::getLength(int8_t occupancyGrid) {
    // TODO
    ROS_WARN("getLength in Class Ray is not yet implemented");
    return 0;
}

Vec2 Ray::getCenter() {
    // TODO
    ROS_WARN("getCenter in Class Ray is not yet implemented");
    return Vec2(0, 0);
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
 * The vectors representing the direction will hold unit circle values
 */
RayTracer::RayTracer(int numberOfRays){
    // numberOfRays is to low
    if(numberOfRays < 1){
        throw std::range_error("numberOfRays in RayTracer::Raytracer() is out of Range");
    }

    std::vector<Ray> rayVec;
    for(int i = 1; i <= numberOfRays; i++){
        /*
         * all rays should be equally distanced, so coordinates in the unit circle will be calculated
         */
        float angle = ((float)i/(float)(numberOfRays + 1) * 180);
        Vec2 dir(cos(angle),sin(angle));
        Ray ray(Vec2(RAYTRACER_RAY_START_X, RAYTRACER_RAY_START_Y),dir);
        rayVec.push_back(ray);
    }
    this->rays = rayVec;

}

const std::vector<Ray> &RayTracer::getRays() const {
    return rays;
}

void RayTracer::addRay(const Ray &ray) {
    // TODO
    ROS_WARN("addRay in Class RayTracer is not yet implemented");
}

void RayTracer::removeRay(int index) {
    // TODO
    ROS_WARN("removeRay in Class RayTracer is not yet implemented");
}

Ray RayTracer::getLongestRay() {
    // TODO
    ROS_WARN("getLongestRay in Class RayTracer is not yet implemented");
    return Ray(Vec2(0,0), Vec2(0,0));
}






