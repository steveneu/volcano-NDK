//
// Created by Steve on 1/3/2017.
//

#include "Matrix3x3.h"
#include <android/log.h>

#define  LOG_TAG    "particle-port"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

void Matrix3x3::copym(const Matrix3x3& rhs) {
    m[ 0] = rhs.m[ 0]; // column 0
    m[ 1] = rhs.m[ 1];
    m[ 2] = rhs.m[ 2];
    m[ 3] = rhs.m[ 3];
    m[ 4] = rhs.m[ 4]; // column 1, etc.
    m[ 5] = rhs.m[ 5];
    m[ 6] = rhs.m[ 6];
    m[ 7] = rhs.m[ 7];
    m[ 8] = rhs.m[ 8]; //
    m[ 9] = rhs.m[ 9];
    m[10] = rhs.m[10];
    m[11] = rhs.m[11];
    m[12] = rhs.m[12]; //
    m[13] = rhs.m[13];
    m[14] = rhs.m[14];
    m[15] = rhs.m[15];
}

Matrix3x3::Matrix3x3() {
    m[0] = m[1] = m[2] = m[3] =
    m[4] = m[5] = m[6] = m[7] =
    m[8] = m[9] = m[10] = m[11] =
    m[12] = m[13] = m[14] = m[15] = 0;
}

Matrix3x3::Matrix3x3(const Matrix3x3& m_in) {
    if (this != &m_in) {
        copym(m_in);
    }
}

Matrix3x3::~Matrix3x3() {
}

Matrix3x3::Matrix3x3(float min[16]) {
    m[ 0] = min[ 0];
    m[ 1] = min[ 1];
    m[ 2] = min[ 2];
    m[ 3] = min[ 3];

    m[ 4] = min[ 4];
    m[ 5] = min[ 5];
    m[ 6] = min[ 6];
    m[ 7] = min[ 7];

    m[ 8] = min[ 8];
    m[ 9] = min[ 9];
    m[10] = min[10];
    m[11] = min[11];

    m[12] = min[12];
    m[13] = min[13];
    m[14] = min[14];
    m[15] = min[15];
}

Matrix3x3::Matrix3x3(const Vec3& col1, const Vec3& col2, const Vec3& col3) {
    m[0] = col1.geta();
    m[1] = col1.getb();
    m[2] = col1.getc();
    m[3] = 0;

    m[4] = col2.geta();
    m[5] = col2.getb();
    m[6] = col2.getc();
    m[7] = 0;

    m[8] = col3.geta();
    m[9] = col3.getb();
    m[10] = col3.getc();
    m[11] = 0;

    m[12] = 0;
    m[13] = 0;
    m[14] = 0;
    m[15] = 1;
}

Matrix3x3& Matrix3x3::operator=(const Matrix3x3& rhs) {
    if (this != &rhs) {
        copym(rhs);
    }
    return *this;
}

// overwrite this matrix with the identity matrix.  this destroys the current values
void Matrix3x3::makeIdentity(void) {
    Matrix3x3();
    m[0] = m[5] = m[10] = m[15] = 1.0;
}

// make this matrix a translation matrix that performs the translation
// by a, b, c in the x-, y-, and z-directions, respectively.
void Matrix3x3::makeTranslation(float a, float b, float c) {
    makeIdentity();
    m[12] = a;  // last column in matrix is the vector <a, b, c, 1>
    m[13] = b;
    m[14] = c;
}

// set the matrix element directly based on its index
void Matrix3x3::set(int idx, float newvalue) {
    m[idx] = newvalue;
}

// copy float array (row major format) into 'this' matrix.
// store and convert to column major
void Matrix3x3::put(float* in) {
    int pos = 0;
    for (int i=0; i<16; i++) {
        m[i] = in[i];
    }
}

// return float array of elements in column major format for OpenGL
float* Matrix3x3::get(void) {
    return m;
}

// get zero based row 'r' from matrix(this) and return in param 'result' as Vec3
void Matrix3x3::getRow(Vec3& result, int r) const {
    if (r<4 && r>= 0)    // sanity check
    {
        int current = r;
    //    Vec3 result(m[current], m[current+4], m[current+4], m[current+4]);
        result.seta(m[current]);    current += 4;
        result.setb(m[current]);    current += 4;
        result.setc(m[current]);    current += 4;
        result.setd(m[current]);
    }
}

void Matrix3x3::getCol(Vec3& result, int c) const {
    int current = c*4;
    result.seta(m[current++]);
    result.setb(m[current++]);
    result.setc(m[current++]);
    result.setd(m[current]);
}

// multiply this matrix by the supplied 3D vector (rhs).  use homogeneous
// transformation
void Matrix3x3::mul(Vec3& result, const Vec3& rhs) {
//    float v3 = 1.0; // homogeneous coordinate for vector

    float res0 = 0;
    float res1 = 0;
    float res2 = 0;
    float res3 = 0;

    res0 = m[0] * rhs.geta() + m[4] * rhs.getb() + m[8] * rhs.getc() + m[12] * rhs.getd();
    res1 = m[1] * rhs.geta() + m[5] * rhs.getb() + m[9] * rhs.getc() + m[13] * rhs.getd();
    res2 = m[2] * rhs.geta() + m[6] * rhs.getb() + m[10] * rhs.getc() + m[14] * rhs.getd();
    res3 = m[3] * rhs.geta() + m[7] * rhs.getb() + m[11] * rhs.getc() + m[15] * rhs.getd();

    result.seta(res0);
    result.setb(res1);
    result.setc(res2);
    result.setd(res3);
}

// multiply lhs matrix (rhs) by the supplied 3D vector (rhs).  return result
void Matrix3x3::mul(Vec3& result, const Matrix3x3& lhs, const Vec3& rhs) {
//    float v3 = 1.0; // homogeneous coordinate for vector

    Vec3 row0, row1, row2, row3;
    lhs.getRow(row0, 0);
    lhs.getRow(row1, 1);
    lhs.getRow(row2, 2);
    lhs.getRow(row3, 3);

    float res0 = Vec3::dot(row0, rhs);
    float res1 = Vec3::dot(row1, rhs);
    float res2 = Vec3::dot(row2, rhs);
    float res3 = Vec3::dot(row3, rhs);

    result.seta(res0);
    result.setb(res1);
    result.setc(res2);
    result.setd(res3);
}

// multiply first matrix by second and return product matrix
void Matrix3x3::mul(Matrix3x3& result, const Matrix3x3& first, const Matrix3x3& second) {
    int index = 0;
    for(int r = 0; r<4; r++) {
        Vec3 row;
        first.getRow(row, r);
        index = r;
        for(int c=0; c<4; c++) {
            Vec3 col;
            second.getCol(col, c);
            float dot = Vec3::dot(row, col);
            result.set(index, dot);
            index += 4;
        }
    }
}

// construct view matrix given eye point, aim point(center) and camera up vector
void Matrix3x3::setLookAt(Matrix3x3& result, const Point& eye, const Point& center, const Vec3& up)
{
    Vec3 forward(center.getx() - eye.getx(),
                 center.gety() - eye.gety(),
                 center.getz() - eye.getz()
    );

    forward.normalize();

    Vec3 side(0,0,0);
    side = Vec3::cross(forward, up);
    Vec3 up2 = Vec3::cross(side, forward);

    Matrix3x3 i;
    i.makeIdentity();

    Vec3 col3(-forward.geta(), -forward.getb(), -forward.getc());
    Matrix3x3 m(side, up2, col3);

    Matrix3x3 c;
    mul(c, i, m);
    Matrix3x3 translation;
    translation.makeTranslation(eye.getx(), eye.gety(), eye.getz());

    mul(result, c, translation);
}

// input: right, left, top, bottom, near, far
// todo: test this vs. java impl. of matrix.frustumM()
void Matrix3x3::frustum(Matrix3x3& result, float l, float r, float b, float t, float n, float f) {
    Vec3 c0;                                        // 1st column
    c0.seta((2*n)/(r-l));
    c0.setb(0);
    c0.setc(0);
    c0.setd(0);

    Vec3 c1;                                        // 2nd col, etc.
    c1.seta(0);
    c1.setb((2*n)/(t-b));
    c1.setc(0);
    c1.setd(0);

    Vec3 c2;
    c2.seta((r+l)/(r-l));
    c2.setb((t+b)/(t-b));
    c2.setc(-1*(f+n)/(f-n));
    c2.setd(-1);

    Vec3 c3;
    c3.seta(0);
    c3.setb(0);
    c3.setc((-2*f*n)/(f-n));
    c3.setd(0);

    result.setcol(0, c0);
    result.setcol(1, c1);
    result.setcol(2, c2);
    result.setcol(3, c3);
}

// set column indicated by param c to v
void Matrix3x3::setcol(int c, const Vec3& v) {
    int offset = c*4;
    m[offset] = v.geta();
    m[offset+1] = v.getb();
    m[offset+2] = v.getc();
    m[offset+3] = v.getd();
}

void Matrix3x3::debugPrint(bool debugPrint, const char* matrix_id) {
    if (debugPrint) {
        LOGI("%s: \n", matrix_id);
        LOGI("\t%.3f\t%.3f\t%.3f\t%.3f", m[0], m[4], m[8], m[12]);
        LOGI("\t%.3f\t%.3f\t%.3f\t%.3f", m[1], m[5], m[9], m[13]);
        LOGI("\t%.3f\t%.3f\t%.3f\t%.3f", m[2], m[6], m[10], m[14]);
        LOGI("\t%.3f\t%.3f\t%.3f\t%.3f", m[3], m[7], m[11], m[15]);
    }
}