#include "pointer.h"
#include <cmath>
#include <iostream>

Pointer::Pointer()
: x(0), y(0)
{
}

void Pointer::update(int x1, int y1)
{
    x = x1;
    y = y1;
}

void Pointer::moveByAngle(float angle, float distance)
{
    const float angleRotated = angle + 1.57079;

    x += -sin(angleRotated) * distance;
    y += cos(angleRotated) * distance;
    std::cout << "angle: " << angle << " anglerotate : " << angleRotated << std::endl;
    std::cout << "sin angle: " << sin(angleRotated) << " cos angle : " << cos(angleRotated) << std::endl;
}

bool Pointer::equalsTo(int x1, int y1)
{
    return (x==x1 && y==y1);
}

float Pointer::getDistanceTo(int x1, int y1)
{
    int dif_x, dif_y;
    dif_x = x - x1;
    dif_y = y - y1;
    dif_x *= dif_x;
    dif_y *= dif_y;
    std::cout << "x, x1 : " << x << " " << x1 << std::endl;
    std::cout << "y, y1 : " << y << " " << y1 << std::endl;
    std::cout << "x : " << dif_x << " y : " << dif_y << std::endl;
    return sqrt(dif_x + dif_y);
}

float Pointer::getAngleTo(int x1, int y1)
{
    int dif_x, dif_y;
    dif_x = x - x1;
    dif_y = y - y1;
    return atan2(dif_y, dif_x);
}