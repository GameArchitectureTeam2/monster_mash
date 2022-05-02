/*
Pointer class used to draw smooth line

It calculate position of brush by taking {x,y} position of pointer.
Brush will only move when pointer go out of the range calculated by radius and x,y position.

*/

#ifndef POINTER_H
#define POINTER_H

class Pointer{
public:
    Pointer();
    int x;
    int y;

    void update(int x1, int y1);
    void moveByAngle(float angle, float distance);
    bool equalsTo(int x1, int y1);
    float getDistanceTo(int x1, int y1);
    float getAngleTo(int x1, int y1);

private:
};

#endif