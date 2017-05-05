//
// Created by Steve on 12/28/2016.
//

#ifndef PARTICLE_PORT_STUDIO_POINT_H
#define PARTICLE_PORT_STUDIO_POINT_H

// represents a point in Cartesian R^3 or R^2 space
class Vec3;

class Point {
    float x, y, z;
    float r, g, b, a; // color

public:
    Point();
    Point(float x1, float y1, float z1);
    void add_vector(Vec3 v); // add the given vector to point, overwriting current values

    // return distance as a float between this.x,y,z and Point p2 in R^3 using distance formula
    float get_distance(Point p2);

    float getr() const;
    float getg() const;
    float getb() const;
    float geta() const;

    float getx() const;
    float gety() const;
    float getz() const;

    void putx(float xval);
    void puty(float yval);
    void putz(float zval);

    void putb(float bval);

    void putcolor(float fr, float fg, float fb, float fa);
};

#endif //PARTICLE_PORT_STUDIO_POINT_H
