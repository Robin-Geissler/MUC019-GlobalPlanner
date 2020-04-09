#ifndef GLOBAL_PLANNER_RAYTRACER_HPP
#define GLOBAL_PLANNER_RAYTRACER_HPP

#include <vector>
#include <iostream>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

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
    Vec2 operator *(const float& factor);


};

class Ray{
private:
    Vec2 start; // start point of the Ray
    Vec2 dir;   // direction of the Ray
    std::vector<Vec2> occuGridFields; // the Ray fields in the occupancyGrid (Vec2 as x,y coordinate)



public:
    Ray(const Vec2 &start, const Vec2 &dir);
    virtual ~Ray();
    const Vec2 &getStart() const;
    void setStart(const Vec2 &start);
    const Vec2 &getDir() const;
    void setDir(const Vec2 &dir);
    void setOccuGridFields(nav_msgs::OccupancyGrid& inputGrid);

    float getLength();
    Vec2 getCenter();
};

class RayTracer{
private:
    std::vector<Ray> rays;  // holds the Rays of the RayTracer
    nav_msgs::OccupancyGrid inputGrid;  // input Occupancy Grid to calculate Ray collisions
    nav_msgs::OccupancyGrid outputGrid; // output Occupancy Grid where the Goal Point is set to 5


public:
    explicit RayTracer(int numberOfVectors);
    const std::vector<Ray> &getRays() const;
    Ray getLongestRay();
    const nav_msgs::OccupancyGrid &getInputGrid() const;
    void setInputGrid(const nav_msgs::OccupancyGrid &inputGrid);
    const nav_msgs::OccupancyGrid &getOutputGrid() const;
    void setOutputGrid(const nav_msgs::OccupancyGrid &outputGrid);

};
#endif //GLOBAL_PLANNER_RAYTRACER_HPP
