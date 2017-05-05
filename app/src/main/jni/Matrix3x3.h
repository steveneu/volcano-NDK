//
// Created by Steve on 1/3/2017.
//

#ifndef PARTICLE_PORT_STUDIO_MATRIX3X3_H
#define PARTICLE_PORT_STUDIO_MATRIX3X3_H

#include "Point.h"
#include "Vec3.h"

// represents a 3x3 matrix with homogeneous representation (4x4) in
// column major format (OpenGL)
class Matrix3x3 {
    float m[16];
    void copym(const Matrix3x3& rhs);
public:
    Matrix3x3();
    Matrix3x3(const Matrix3x3& m_in);
    Matrix3x3(float min[16]);
    Matrix3x3(const Vec3& col1, const Vec3& col2, const Vec3& col3);

    ~Matrix3x3();
    Matrix3x3 & operator=(const Matrix3x3& rhs);

    void set(int idx, float newvalue);
    void setcol(int c, const Vec3& v);
    void put(float* in);
    float* get(void);
    void makeIdentity(void);
    void makeTranslation(float a, float b, float c);
    void getRow(Vec3& result, int r) const;
    void getCol(Vec3& result, int c) const;
    void mul(Vec3& result, const Vec3& rhs);
    void debugPrint(bool debugPrint, const char* matrix_id);
    static void mul(Vec3& result, const Matrix3x3& lhs, const Vec3& rhs);
    static void mul(Matrix3x3& result, const Matrix3x3& first, const Matrix3x3& second);
    static void setLookAt(Matrix3x3& result, const Point& eye, const Point& center, const Vec3& up);
    static void frustum(Matrix3x3& result, float left, float right, float bottom, float top, float near, float far);
};


#endif //PARTICLE_PORT_STUDIO_MATRIX3X3_H
