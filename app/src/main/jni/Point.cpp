//
// Created by Steve on 12/28/2016.
//

#include "Point.h"
#include "Vec3.h"
#include <math.h>

// represents a point in Cartesian R^3 or R^2 space
Point::Point() {
    x= y= z= 0;
    r= g= b= a= 0;
}

Point::Point(float x1, float y1, float z1) {
    x = x1;
    y = y1;
    z = z1;
    r= g= b= a= 0;
}

void Point::add_vector(Vec3 v) // add the given vector to point, overwriting current values
{
    x += v.geta();
    y += v.getb();
    z += v.getc();
}

// return distance as a float between this.x,y,z and Point p2 in R^3 using distance formula
float Point::get_distance(Point p2) {
    return sqrt(pow(p2.getx()-x,2.0f) + pow(p2.gety()-y,2.0f) + pow(p2.getz()-z,2.0f));
}

float Point::getr() const { return r; }
float Point::getg() const { return g; }
float Point::getb() const { return b; }
float Point::geta() const { return a; }

float Point::getx() const { return x; }
float Point::gety() const { return y; }
float Point::getz() const { return z; }

void Point::putx(float xval) { x=xval; }
void Point::puty(float yval) { y=yval; }
void Point::putz(float zval) { z=zval; }

void Point::putb(float bval) { b=bval; }

void Point::putcolor(float fr, float fg, float fb, float fa) {
    r = fr;
    g = fg;
    b = fb;
    a = fa;
}


