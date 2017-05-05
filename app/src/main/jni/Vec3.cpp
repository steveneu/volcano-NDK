//
// Created by Steve on 12/28/2016.
//

#include "Vec3.h"
#include "Point.h"
#include <android/log.h>

#define  LOG_TAG    "particle-port"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

Vec3::Vec3() {
    a = b = c = 0;
    d = 1;
}

Vec3::Vec3(const Vec3& v) {
    a = v.a;  b = v.b;  c = v.c;  d = v.d;
}

Vec3::Vec3(float aval, float bval, float cval, float dval/*=1.0*/) {
    a= aval; b= bval; c= cval; d=dval;
}

Vec3::Vec3(Point p1, Point p2) { // make a vector given 2 points
    a = p2.getx()-p1.getx();
    b = p2.gety()-p1.gety();
    c = p2.getz()-p1.getz();
    d = 1.0;
}

Vec3 Vec3::scale(int scalefactor) {
    //= new Vec3(a*scalefactor, b*scalefactor, c*scalefactor);
    Vec3 scaled(a*scalefactor, b*scalefactor, c*scalefactor);
    return scaled;
}

void Vec3::set(float aIn, float bIn, float cIn, float dIn/*=1.0*/) {
    a = aIn; b= bIn; c= cIn; d = dIn;
}

float Vec3::geta() const { return a; }
float Vec3::getb() const { return b; }
float Vec3::getc() const { return c; }
float Vec3::getd() const { return d; }

void Vec3::seta(float aIn) { a = aIn; }
void Vec3::setb(float bIn) { b = bIn; }
void Vec3::setc(float cIn) { c = cIn; }
void Vec3::setd(float dIn) { d = dIn; }

void Vec3::normalize() {
    float r = sqrt( a*a + b*b + c*c );
    if (r == 0.0)
        return;

    a /= r;
    b /= r;
    c /= r;
}

// returns 3D cross product.  result = first X second;
Vec3 Vec3::cross(const Vec3& first, const Vec3& second) {
    Vec3 result;

    result.set(first.getb() * second.getc() - first.getc() * second.getb(),
        first.getc() * second.geta() - first.geta() * second.getc(),
        first.geta() * second.getb() - first.getb() * second.geta()
    );

    return result;
}

float Vec3::dot(const Vec3& first, const Vec3& second) {
    Vec3 result;

    return first.geta() * second.geta() + first.getb() * second.getb() +
            first.getc() * second.getc() + first.getd() * second.getd();
}

//void Matrix3x3::debugPrint(bool debugPrint, const char* matrix_id) {
void Vec3::debugPrint(bool debugPrint, const char* vec_string) {
    if (debugPrint) {
        LOGI("%s: \n", vec_string);
        LOGI("\t%.3f\t%.3f\t%.3f\t%.3f", a, b, c, d);
    }
}

