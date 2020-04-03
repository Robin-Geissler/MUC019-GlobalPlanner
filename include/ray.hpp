#ifndef GLOBAL_PLANNER_RAY_HPP
#define GLOBAL_PLANNER_RAY_HPP

#include <vector>
#include <iostream>
#include <cmath>

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
    float length; // length of the Ray (until intersection)

public:
    Ray(const Vec2 &start, const Vec2 &dir);

};

#endif //GLOBAL_PLANNER_RAY_HPP
