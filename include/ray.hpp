#ifndef GLOBAL_PLANNER_RAY_HPP
#define GLOBAL_PLANNER_RAY_HPP

#include <vector>
#include <iostream>

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



};

#endif //GLOBAL_PLANNER_RAY_HPP
