#include "rayTracer.hpp"

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
    return 0;
}

Vec2 Ray::getCenter() {
    // TODO
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
}

void RayTracer::deleteRay(int index) {
    // TODO
}

Ray RayTracer::getLongestRay() {
    // TODO
    return Ray(Vec2(0,0), Vec2(0,0));
}




