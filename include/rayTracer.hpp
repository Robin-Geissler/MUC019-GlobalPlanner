#ifndef GLOBAL_PLANNER_RAYTRACER_HPP
#define GLOBAL_PLANNER_RAYTRACER_HPP

#include <vector>
#include <iostream>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#define RAYTRACER_RAY_START_X 1
#define RAYTRACER_RAY_START_Y 0
#define RAYTRACER_RAYNUMBER 25  // should always be odd

#define RAY_BOOST_SCALOR 1  ///< Scales the importance of driving straight by adding weight to the inner rays in RayTracer::getBestRay()
#define RAY_CRITICAL_COLLISION_PROBABILITY 0    ///< Fields in the occupancy grid with a probability greater then this, will serve as ray blockers

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

    Coordinate start; ///< Start point of the Ray

    Vec2 dir;   ///< Direction of the Ray

    std::vector<Coordinate> occuGridFields; ///< The Ray fields in the occupancyGrid (Vec2 as x,y coordinate)

    /*
     * Helper functions
     */

    /**
     * @brief Checks if the field is out of bounds
     * @param field The grid coordinate that is to be checked
     * @param grid The occupancy grid
     * @return True if field is out of Bounds false if not
     */
    static bool outOfBounds(Coordinate field, const nav_msgs::OccupancyGrid& grid);

    /**
     * @brief Checks if the field is at an edge of the grid
     * @param field The grid coordinate that is to be checked
     * @param grid The occupancy grid
     * @return True if field is on the grid edge false if not
     */
    static bool edge(Coordinate field, const nav_msgs::OccupancyGrid& grid);

    /**
     * @brief Checks if a field in grid is occupied
     * @param field The grid coordinate that is to be checked
     * @param grid The occupancy grid
     * @return True if field is occupied false if not
     */
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

    /**
     * @brief Calculates and sets the OccuGridFields of the Ray, depending on the inputGrid.
     * The first GridPoint of the Ray will be its start Coordinate.
     * From that GridPoint on the algorithm will generally follow the dir of the Vector.
     * The in absolute bigger coordinate of dir will be the mainDirection.
     * The in absolute smaller coordinate of dir will be the subDirection.
     * Dir is scaled, so that the subDirection will have an absolute value of 1.
     * The algorithm will go:
     * 1. into the mainDirection the half way.
     * 2. into the subDirection one step.
     * 3. into the mainDirection the other half way.
     * Then start from the beginning again.
     * The algorithm ends if the end of the grid is reached, or the next GridPoint is occupied.
     * From which value on a GridPoint will block Rays can be set with the macro RAY_CRITICAL_COLLISION_PROBABILITY
     * @param inputGrid The occupancyGrid where the Ray lies in
     */
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

    /**
     * @brief Calculates the grid-array-index of field.
     * @param field Coordinate which´s index is to be calculated
     * @param grid OccupancyGrid with the grid-array
     * @return Grid-array-index of field
     */
    static int getCoodinateIndex(Coordinate field, const nav_msgs::OccupancyGrid& grid);
};

class RayTracer{
private:
    std::vector<Ray> rays;  ///< Holds the Rays of the RayTracer
    nav_msgs::OccupancyGrid inputGrid;  ///< Input Occupancy Grid from Lidar to calculate Ray collisions
    nav_msgs::OccupancyGrid outputGrid; ///< Output Occupancy Grid where the Goal Point is set to 5




public:

    RayTracer();

    /**
    * @brief This is the Constructor for the RayTracer.
    * It Constructs numberOfRays Rays.
    * All Rays will start at RAYTRACER_RAY_START_X and RAYTRACER_RAY_START_Y.
    * Direction of the generated Rays:
    * The direction of the Rays will be equally distributed between 0 and 180 degrees.
    * 0 degrees and 180 degrees are not included.
    * The Vector will be scalled, so that the abs value of the coordinate with lower abs value will be 1
    * @param numberofRays Number of rays generated by the Ray Tracer
    * @param gridWidth Width of the incoming and outgoing Occupany Grids
    * @param gridHeight Height of the incoming and outgoing Occupany Grids
    */
    explicit RayTracer(int numberOfRays, int gridWidth, int gridHeight);

    /**
     *
     * @return All rays of the RayTracer
     */
    const std::vector<Ray> &getRays() const;

    /**
     * @brief Calculates the current best ray of the RayTracer.
     * The value for all Rays will be calculated, the Ray with the best value will be returned.
     * The value of a ray is mainly set by its length.
     * There is a bonus on "how centered the ray is in the grid(straight direction is max bonus)".
     * The weight of that bonus can be set by the macro RAY_BOOST_SCALOR.
     * Generally a inner Ray gets a bonus of: NextOuterRayBoost + RAY_BOOST_SCALOR.
     * The most outer rays get no bonus (bonus of 0).
     * @return Current best ray of RayTracer
     */
    Ray getBestRay();

    const nav_msgs::OccupancyGrid &getInputGrid() const;

    /**
     * @brief Updates the inputGrid.
     * @param inputGrid The new inputGrid
     */
    void setInputGrid(const nav_msgs::OccupancyGrid &inputGrid);

    /**
     * @brief Returns the output grid of the RayTracer
     * @return The outputGrid of the RayTracer
     */
    const nav_msgs::OccupancyGrid &getOutputGrid() const;

    /**
     * @brief Sets occuGridFields for all Rays according to the inputGrid, and updates the outputGrid.
     * Sets middle Point of the best Ray to 100 everything else to 0.
     */
    void setOutputGrid();

};
#endif //GLOBAL_PLANNER_RAYTRACER_HPP
