#ifndef GLOBAL_PLANNER_RAYTRACER_HPP
#define GLOBAL_PLANNER_RAYTRACER_HPP

#include <vector>
#include <iostream>
#include <cmath>

#define RAYTRACER_RAY_START_X 50
#define RAYTRACER_RAY_START_Y 0
#define RAYTRACER_RAYNUMBER 25  // should always be odd

class Vec2{
private:
    float x;
    float y;

public:
    Vec2(float x, float y);
    virtual ~Vec2();
    float getX() const;
    void setX(float x);
    float getY() const;
    void setY(float y);
    Vec2 operator +(const Vec2& vec);
    Vec2 operator -(const Vec2& vec);


};

class Ray{
private:
    Vec2 start; // start point of the Ray
    Vec2 dir;   // direction of the Ray

public:
    Ray(const Vec2 &start, const Vec2 &dir);
    virtual ~Ray();
    const Vec2 &getStart() const;
    void setStart(const Vec2 &start);
    const Vec2 &getDir() const;
    void setDir(const Vec2 &dir);

    float getLength(int8_t occupancyGrid);
    Vec2 getCenter();
};

class RayTracer{
private:
    std::vector<Ray> rays;

public:
    explicit RayTracer(int numberOfVectors);
    const std::vector<Ray> &getRays() const;
    void addRay(const Ray &ray);
    void removeRay(int index);
    Ray getLongestRay();

};
#endif //GLOBAL_PLANNER_RAYTRACER_HPP
