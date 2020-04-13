#ifndef GLOBAL_PLANNER_RAYTRACER_HPP
#define GLOBAL_PLANNER_RAYTRACER_HPP

#include <vector>
#include <iostream>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#define RAYTRACER_RAY_START_X 50
#define RAYTRACER_RAY_START_Y 0
#define RAYTRACER_RAYNUMBER 25  // should always be odd

class Coordinate{
private:
    int x;
    int y;
public:
    Coordinate(int x, int y);
    int getX() const;
    void setX(int x);
    int getY() const;
    void setY(int y);
    Coordinate operator +(const Coordinate& coo);
    Coordinate operator -(const Coordinate& coo);
    Coordinate operator *(const float& factor);
    Coordinate operator /(const float& divisor);

};

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
    Coordinate start; // start point of the Ray
    Vec2 dir;   // direction of the Ray
    std::vector<Coordinate> occuGridFields; // the Ray fields in the occupancyGrid (Vec2 as x,y coordinate)
    std::tuple<int,int> hi;

    /*
     * Helper functions
     */
    static int getCoodinateIndex(Coordinate field, const nav_msgs::OccupancyGrid grid);
    bool outOfBounds(Coordinate field, nav_msgs::OccupancyGrid grid);
    bool occupied(Coordinate field, nav_msgs::OccupancyGrid grid);

    /*
     * breef: decreases abs of num by 1 and outputs the difference
     */
    int decAbs(float *num);


public:
    Ray(const Coordinate &start, const Vec2 &dir);
    virtual ~Ray();
    const Coordinate &getStart() const;
    void setStart(const Coordinate &start);
    const Vec2 &getDir() const;
    void setDir(const Vec2 &dir);
    void setOccuGridFields(nav_msgs::OccupancyGrid& inputGrid);

    // just public for testing for the moment
    /*
     * breef: gos into mainDir if possible, subDir else. Adjusts all Input accordingly
     */
    Coordinate getNextGridPoint(float *mainDirNextSteps, float *subDirNextSteps, int *mainDirCoordinate, int *subDirCoordinate);

    float getLength();
    Coordinate getCenter();
};

class RayTracer{
private:
    std::vector<Ray> rays;  // holds the Rays of the RayTracer
    nav_msgs::OccupancyGrid inputGrid;  // input Occupancy Grid from Lidar to calculate Ray collisions
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
