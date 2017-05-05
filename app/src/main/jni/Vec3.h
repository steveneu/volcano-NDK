//
// Created by Steve on 12/28/2016.
//

#ifndef PARTICLE_PORT_STUDIO_VEC3_H
#define PARTICLE_PORT_STUDIO_VEC3_H

#include "Point.h"
#include <math.h>

class Point;

// represents a vector with basis at origin, does not represent a point in space
class Vec3 {

    float a, b, c, d;
public:
    Vec3();
    Vec3(const Vec3& v);
    Vec3(float aval, float bval, float cval, float dval=1.0);

    Vec3(Point p1, Point p2);
    Vec3 scale(int scalefactor);

    void set(float aIn, float bIn, float cIn, float dIn=1.0);

    void seta(float aIn);
    void setb(float bIn);
    void setc(float cIn);
    void setd(float dIn);

    float geta() const;
    float getb() const;
    float getc() const;
    float getd() const;

    void normalize();

    static Vec3 cross(const Vec3& first, const Vec3& second);
    static float dot(const Vec3& first, const Vec3& second);
    void debugPrint(bool debugPrint, const char* vec_string);
};

#endif //PARTICLE_PORT_STUDIO_VEC3_H
