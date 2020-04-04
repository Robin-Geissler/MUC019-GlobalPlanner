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




