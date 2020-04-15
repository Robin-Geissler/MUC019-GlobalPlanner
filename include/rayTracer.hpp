#ifndef GLOBAL_PLANNER_RAYTRACER_HPP
#define GLOBAL_PLANNER_RAYTRACER_HPP

#include <vector>
#include <iostream>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#define RAYTRACER_RAY_START_X 1
#define RAYTRACER_RAY_START_Y 0
#define RAYTRACER_RAYNUMBER 25  // should always be odd

#define RAY_BOOST_SCALOR 1  // scales the importance of driving straight by removing weight from outer rays in RayTracer::getBestRay
#define RAY_CRITICAL_COLLISION_PROBABILITY 0    // Fields with this probability in the occupancy grid will serve as ray blockers

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

    /*
     * Helper functions
     */
    static int getCoodinateIndex(Coordinate field, const nav_msgs::OccupancyGrid& grid);

    static bool outOfBounds(Coordinate field, const nav_msgs::OccupancyGrid& grid);

    static bool edge(Coordinate field, const nav_msgs::OccupancyGrid& grid);

    static bool occupied(Coordinate field, const nav_msgs::OccupancyGrid& grid);

    /**
     * @brief decreases abs of num by 1 and outputs the difference
     * @param num the number that should be decreased
     * @return the difference between the old and the new number
     */
    int decAbs(float *num);


public:

    Ray(const Coordinate &start, const Vec2 &dir);

    virtual ~Ray();

    const Coordinate &getStart() const;

    void setStart(const Coordinate &start);

    const Vec2 &getDir() const;

    void setDir(const Vec2 &dir);

    void setOccuGridFields(const nav_msgs::OccupancyGrid& inputGrid);

    // just public for testing for the moment
    /**
     * @brief go´s into mainDir if possible, subDir else. Adjusts all Input accordingly.
     * The next grid coordinate will be saved in mainDir Coordinate and subDirCoordinate
     * @param mainDirNextSteps the number of steps that need to be taken in the mainDir
     * @param subDirNextSteps the number of steps that need to be taken in the subDir
     * @param mainDirCoordinate the current Coordinate of mainDir
     * @param subDirCoordinate the current Coordinate of subDir
     */
    void getNextGridPoint(float *mainDirNextSteps, float *subDirNextSteps, int *mainDirCoordinate, int *subDirCoordinate);

    float getLength(const nav_msgs::OccupancyGrid& inputGrid);
    Coordinate getCenter();
};

class RayTracer{
private:
    std::vector<Ray> rays;  ///  holds the Rays of the RayTracer
    nav_msgs::OccupancyGrid inputGrid;  /// input Occupancy Grid from Lidar to calculate Ray collisions
    nav_msgs::OccupancyGrid outputGrid; /// output Occupancy Grid where the Goal Point is set to 5



public:
    /**
    * @brief: this is the Constructor for the RayTracer
    * It Constructs numberOfRays Rays.
    * All Rays will start at RAYTRACER_RAY_START_X and RAYTRACER_RAY_START_Y.
    * Direction of the generated Rays:
    * The direction of the Rays will be equally distributed between 0 and 180 degrees.
    * 0 degrees and 180 degrees are not included.
    * The Vector will be scalled, so that the abs value of the coordinate with lower abs value will be 1
    * @param numberofRays number of rays generated by the Ray Tracer
    */
    explicit RayTracer(int numberOfVectors);

    const std::vector<Ray> &getRays() const;

    Ray getBestRay();

    const nav_msgs::OccupancyGrid &getInputGrid() const;

    /**
     * @brief updates the inputGrid, and sets occuGridFields for all Rays accordingly
     * @param inputGrid the new inputGrid
     */
    void setInputGrid(const nav_msgs::OccupancyGrid &inputGrid);
    /**
     * @brief returns the output grid of the RayTracer
     * @return the outputGrid of the RayTracer
     */
    const nav_msgs::OccupancyGrid &getOutputGrid() const;

    void setOutputGrid(const nav_msgs::OccupancyGrid &outputGrid);

};
#endif //GLOBAL_PLANNER_RAYTRACER_HPP
