/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// OpenGL ES 2.0 code
#include <jni.h>

//#include "com_neusoft_particle_ObjectJNI.h" // includes <jni.h>
#include <android/log.h>
#include <vector>
#include <algorithm>
#include <time.h>

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Point.h"
#include "Vec3.h"
#include "Matrix3x3.h"
#include "ResizingArray.h"

#define  LOG_TAG    "particle-port"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
//#define  LOGF(...)  __android_log_print(ANDROID_LOG_FATAL,LOG_TAG,__VA_ARGS__)

static const unsigned short TRIANGLE = 0x1; // for sanity check
static const unsigned short OBELISK = 0x2;
static const unsigned short PARTICLES = 0x4;
static const unsigned short GROUND = 0x8;
static const unsigned short TEXTURE = 0x10;
static const unsigned short WIREFRAME = 0x20;        // draw wireframes, if available

static const char* TAG = "Particle";
static const int vCount = 48;              // number of values in vertex array for obelisk
                                            // java equivalent is vertexdata.length
static const int particleCount = 200;
static const bool drawParticles = false;

// return current time in milliseconds
static long now_ms(void) {
    long result = 0;
    struct timespec res;
    clock_gettime(CLOCK_REALTIME, &res);
    result = 1000.0 * res.tv_sec + static_cast<double>(res.tv_nsec) / 1e6;
    return result;
}

static float rnd_float(void)  {
    int random_number = rand();// random();
    float result = static_cast<float>(random_number) / static_cast<float>(RAND_MAX);
    return result;
}

static float toRadians(double degrees) {
    return M_PI/180 * degrees;
}

class AParticle // represents an individual particle
{
    Point previous_position; // previous position
    Point current_position; // current position
    Vec3 direction; // vector that represents the 'force' needed to change the particle position for a specific interval
    Vec3 gravity; // vector representing the force of gravity
    int lifetime; // how long this particle will last in milliseconds
    int lifeleft; // how much life is left in the particle
    float distance_travelled; // how far has this particle travelled?

    float orig_r; float orig_g; float orig_b; float orig_a;
    float final_r; float final_g; float final_b; float final_a;
    float current_r; float current_g; float current_b; float current_a;

    //	private Color cur_color; // current color of particle
    //	private Color last_color; // previous color of particle
    //	private Color delta_color; // change of color
    //	private Point prev; //  previous position

public:
    int get_lifeleft() { return lifeleft; }

    float getr() { return current_r; }
    float getg() { return current_g; }
    float getb() { return current_b; }
    float geta() { return current_a; }

    bool expired() {
        if (lifeleft <=0)
            return true;
        else
            return false;
    }

    void subtract_lifetime(int s) { // subtract s from value in lifetime
        if (lifeleft-s < 0) {
            lifeleft = 0;
        }
        else {
            lifeleft -= s;
        }
    }

    AParticle() {
        current_position.putx(0);
        current_position.puty(0);
        current_position.putz(0);
        lifetime = 0;
        distance_travelled = 0.0f;
        orig_r = orig_g = orig_b = orig_a = 0.0f;
    }

    // regarding color, transition from r,g,b,a to final_r,final_g,final_b,final_a during the life of the particle
    AParticle(float ptx, float pty, float ptz, float dira, float dirb, float dirc,
                     float r, float g, float b, float a, // initial color
                     float f_r, float f_g, float f_b, float f_a, // final color
                     Vec3 gr, int iLifetime)
    {
        current_position.putx(ptx);
        current_position.puty(pty);
        current_position.putz(ptz);

        current_position.putcolor(r,g,b,a);

        // save original and final color for reference when calculating color changes over the life of the particle
        current_r = orig_r = r;		current_g = orig_g = g;   current_b = orig_b = b;   current_a = orig_a = a;
        final_r = f_r; final_g = f_g; final_b = f_b; final_a = f_a;

        direction.seta(dira);
        direction.setb(dirb);
        direction.setc(dirc);

        gravity = gr;
        lifetime = lifeleft = iLifetime;
        distance_travelled=0.0f;
    }

    // change the position of the particle
    void UpdatePosition() {
        previous_position.putx(current_position.getx());
        previous_position.puty(current_position.gety());
        previous_position.putz(current_position.getz());

        current_position.add_vector(direction);
    }

    void applyGravity() {
        // calculate life elapsed in ms
        int lifeElapsed = lifetime - lifeleft;
        // Magnify the gravity vector based on the amount of time gravity has been acting on it
        Vec3 acceleration = gravity.scale(lifeElapsed);
        current_position.add_vector(acceleration);
    }

    void UpdateAttributes() {
        float pct_life = (float)lifeleft/(float)lifetime;

        float color_range_r = final_r - orig_r;
        float color_range_g = final_g - orig_g;
        float color_range_b = final_b - orig_b;
        float color_range_a = final_a - orig_a;

        // change only the b color component for now
        current_r = orig_r + (1-pct_life) * color_range_r;
        current_g = orig_g + (1-pct_life) * color_range_g;
        current_b = orig_b + (1-pct_life) * color_range_b;
        current_a = orig_a + (1-pct_life) * color_range_a;
    }

    void UpdateDistance() {
        distance_travelled += previous_position.get_distance(current_position);
    }

    // get the point to render after it has been influenced by relevant parameters
    Point getRenderPos() {
        //Vec3 v = new Vec3(previous_position, current_position); // direction the particle is currently traveling
        //float displacement = FloatMath.sin(distance_travelled);
        //TODO: Return the current position with a random height (y) displacement added
        return current_position;
    }

    Point getPreviousPos() {
        return previous_position;
    }
};

class sceneManager {
    static const int mode_count =3;
    int current_mode;
    unsigned short drawModes[mode_count];
    unsigned short drawFlags;
    GLuint mProgram_particles;
    GLuint mProgram_texmesh;

    GLint vertex_attrib_idx;
    GLint color_attrib_idx;
    GLint texture_coords_idx;
    GLint texture_uniform;
    GLint textureBool;
    GLint muMVPMatrixHandle;

    Matrix3x3 mMVPMatrix;
    Matrix3x3 mVMatrix;
    Matrix3x3 mProjMatrix;
    float vCameraPos[4]; // = new float[4];
    float vUpVector[4]; // = new float[4];
    float vCurrentUpVector[4]; // = new float[4];

    Matrix3x3 mOrientation;

    std::vector<AParticle> particles;
    int particles_per_sec;  // the mean number of particles generated per interval (second)

    int framesdrawn;        // tracks the frames drawn for fps calculation
    long intervalbegin;
    long updatetime;
    long lastParticleTime;    // the time the last particle was created

    int updates_second;
    int particle_life;
    int arraysize;
    GLuint textureUnit0;              // main texture unit for application

    Vec3 gravity;

    bool windowInitialized;
    bool debugLog;

    float obeliskVertices[48];
    float colordata[4];

    ResizingArray<GLfloat>* volcanoComponents;
    ResizingArray<GLushort>* volcanoIndices;
    ResizingArray<GLfloat>* volcanoColors;

    ResizingArray<GLfloat>* groundComponents;
    ResizingArray<GLfloat>* groundTexComponents;
    ResizingArray<GLushort>* groundIndicesLines;
    ResizingArray<GLushort>* groundIndicesTriangles;
    ResizingArray<GLfloat>* groundColors;

    ResizingArray<GLfloat>* particleComponents;
    ResizingArray<GLushort>* particleIndices;
    ResizingArray<GLfloat>* particleColors;

    // convert indices of 3d array to absolute position of row major array
    inline void assign(GLubyte* array, int row, int col, int pos, GLubyte value, int column_count) {
        int idx = row * column_count + col + pos;
        array[idx] = value;
    }

    void setupVolcanoData();
    void setupGroundData();
    void setupParticleData();

    void drawVolcano();
    void drawGroundNoTexture();
    void drawGroundWithTexture();
    void drawParticles();

    void setTextureProgram();
    void setBasicProgram();
    inline bool flagSet(unsigned short flags, unsigned short bits) {
        if ((flags & bits) == bits)
            return true;
        else
            return false;
    }
public:
    sceneManager();
    ~sceneManager();
    void setIdentity(float* m);
    void changeLookAt();
    void toggleTouchdown();
    void storeOrientation(float* in);
    void printGLString(const char *name, GLenum s);
    void checkGlError(const char* op);
    void setLookAt(Matrix3x3& matrix,			// allocate this before calling with: matrix = new float[16];
                   const Vec3& eye,
                   const Vec3& target,
                   const Vec3& upDir) const;
    void initTextures();
    GLuint loadShader(GLenum shaderType, const char* pSource);
    void createProgram(GLuint* program, const char* pVertexSource, const char* pFragmentSource);
    void createPrograms(const char* pVertexSource, const char* pFragmentSource,
                   const char* pVertexTextureSrc, const char* pFragmentTextureSrc);
    void surfaceChanged(int w, int h);
    void drawFrame();
};

sceneManager::sceneManager() {
    drawFlags = OBELISK | PARTICLES | GROUND; // controls what objects are drawn (obelisk, particles, textures, etc.)
//    drawFlags = OBELISK | PARTICLES | GROUND;
    debugLog = false;

    drawModes[0] = OBELISK | PARTICLES | GROUND ;
    drawModes[1] = OBELISK | PARTICLES | GROUND | WIREFRAME;
    drawModes[2] = OBELISK | PARTICLES | GROUND | TEXTURE;

    current_mode = 2;
    drawFlags = drawModes[current_mode];

    mProgram_particles = mProgram_texmesh = 0;
    vertex_attrib_idx = color_attrib_idx = muMVPMatrixHandle = 0;

    memset(vCameraPos, 0, sizeof(float)*4); // = new float[4];
    memset(vUpVector, 0, sizeof(float)*4); // = new float[4];
    memset(vCurrentUpVector, 0, sizeof(float)*4); // = new float[4];

    colordata[0] = 1.0f;
    colordata[1] = 1.0f;
    colordata[2] = 1.0f;
    colordata[3] = 1.0f;

    particles_per_sec = framesdrawn = intervalbegin = updatetime = lastParticleTime=0;  // the mean number of particles generated per interval (second)
    updates_second = particle_life = 0;
    arraysize= 200;

    windowInitialized = false;
    mOrientation.makeIdentity();

    framesdrawn = 0;

    // todo: get clock values from system for the following:
    long intervalbegin = now_ms();
    long updatetime = now_ms();
    long lastparticletime = now_ms();

    // todo: initialize pseudorandom number generator

    vCameraPos[0] = 0;
    vCameraPos[1] = 0;
    vCameraPos[2] = 3;
    vCameraPos[3] = 1;

    gravity.set(0, 0, -0.000098);

    // this value can affect the trajectory height of the particles.  should be factored into particle position update
    updates_second = 48;

    particles_per_sec = 12; //5
    particle_life = 10; //2000;

    setupVolcanoData();
    setupParticleData();
    setupGroundData();
}

void sceneManager::setupVolcanoData() {
    float obelisk_verts[] = { // 15 elements
            // e: 0f, 0f, 1.0f, - obelisk vertices
            // a: -.5f, .5f, 0,
            // b: .5f, .5f, 0f,
            // c: .5f, -.5f, 0f,
            // d: -.5f, -.5f, 0f,
            -0.5, 0.5, 0.0,     // a, 0
            0.5, 0.5, 0.0,      // b, 1
            0.5, -0.5, 0.0,     // c, 2
            -0.5, -0.5, 0.0,    // d, 3
            0.0, 0.0, 1.0,      // e, 4
    };

    // start with somewhat arbitrary number of 200 particles. particleCount=200
    // 2 vertices per particle (not for obelisk), 3 components per vertex
    // vertices = new float[2 * 3 * (particles.size() + obeliskVerts)];
    int componentCount = 0;

    if (drawFlags & OBELISK || drawFlags & TRIANGLE) {
        componentCount += sizeof(obelisk_verts) / sizeof(*obelisk_verts); // gives # of elements (floats)
        volcanoComponents = new ResizingArray<GLfloat>(componentCount);

        for(int i=0; i<componentCount; i++) {
            volcanoComponents->add(obelisk_verts[i]);
        }
    }

    // indices for GL_LINES drawing
//    GLushort ind[] = {0, 1, 1, 2, 2, 3, 3, 0, // base
//                      0, 4, 1, 4, 2, 4, 3, 4// connect top to base vertices
//    };
    GLushort ind[] = {4,0,3, 4,1,0, 4,2,1, 4,3,2}; // GL_TRIANGLES

    int numIndices = sizeof(ind)/sizeof(*ind);
    volcanoIndices = new ResizingArray<GLushort>(numIndices);

    for(int i=0; i<numIndices; i++) {
        volcanoIndices->add(ind[i]);
    }

    volcanoColors = new ResizingArray<GLfloat>(componentCount * 3);
    GLfloat red=1.0, green=1.0, blue=1.0;

    volcanoColors->add(1.0); //0
    volcanoColors->add(0.0);
    volcanoColors->add(0.0);

    volcanoColors->add(0.0); //1
    volcanoColors->add(1.0);
    volcanoColors->add(0.0);

    volcanoColors->add(0.0); //2
    volcanoColors->add(0.0);
    volcanoColors->add(1.0);

    volcanoColors->add(0.5); //3
    volcanoColors->add(0.0);
    volcanoColors->add(0.5);

    volcanoColors->add(1.0); //4
    volcanoColors->add(1.0);
    volcanoColors->add(1.0);
}

void sceneManager::initTextures() {
    glGenTextures(1, &textureUnit0);
    glBindTexture(GL_TEXTURE_2D, textureUnit0);

    // generate square texture, checkerboard pattern
    const int dimension = 64;
    GLubyte* checkImage = new GLubyte[dimension * dimension * 4];
    int index = 0;

    int c = 0;
    for(int i=0; i<dimension; i++) {
        for(int j=0; j<dimension; j++) {
            c = ( ((i & 0x8) == 0)^((j & 0x8) == 0) ) * 255;
            // inline int sceneManager::assign(GLubyte* array, int row, int col, int pos, GLubyte value, int column_count) {
            checkImage[index++] = static_cast<GLubyte>(c); // red
            checkImage[index++] = static_cast<GLubyte>(c); // grn
            checkImage[index++] = static_cast<GLubyte>(c); // blu
            checkImage[index++] = 0xFF; // alpha
        }
    }

    // make a solid red texture- for testing
//    for(int r=0; r<dimension; r++) {
//        for(int c=0; c<dimension; c++) {
//            checkImage[index++] = 0xFF; // red
//            checkImage[index++] = 0x0; // grn
//            checkImage[index++] = 0x0; // blu
//            checkImage[index++] = 0xFF; // alpha
//        }
//    }

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // copies client memory to GPU texture unit?
    glTexImage2D(GL_TEXTURE_2D,
                 0, // base image level for mipmapping
                 GL_RGBA, // internal format
                 dimension, // width
                 dimension, // height
                 0,  // border width, must be 0
                 GL_RGBA, // format of texels, should match 3rd parameter
                 GL_UNSIGNED_BYTE, //
                 checkImage); // pointer to image data

    // at this point can release memory?
    delete[] checkImage;
    glEnable(GL_CULL_FACE);
}

void sceneManager::setupGroundData() {
    GLfloat ground_verts[] = { // 12 elements
             -2.0,  2.0, 0.0,      // e, 0  // ground vertices
             2.0, 2.0, 0.0,      // f, 1
            2.0, -2.0, 0.0,      // g, 2
            -2.0,  -2.0, 0.0      // h, 3
    };

    int componentCount = sizeof(ground_verts) / sizeof(*ground_verts); // gives # of elements (floats)
    groundComponents = new ResizingArray<GLfloat>(componentCount);

    for(int i=0; i<componentCount; i++) {
        groundComponents->add(ground_verts[i]);
    }

    // store texture coordinates for ground
    GLfloat groundTexCoords[] = {
            0.0, 1.0,
            1.0, 1.0,
            1.0, 0.0,
            0.0, 0.0
    };

    int stCount = sizeof(groundTexCoords) / sizeof(*groundTexCoords);
    groundTexComponents = new ResizingArray<GLfloat>(stCount);
    for(int i=0; i<stCount; i++) {
        groundTexComponents->add(groundTexCoords[i]);
    }

    GLushort indices_lines[] = {0,1, 1,2, 2,3, 3,0, 0,2};   // GL_LINES
    GLushort indices_triangles[] = {0,2,1, 0,3,2}; // GL_TRIANGLES
    int numIndicesLines = 0;
    int numIndicesTriangles = 0;

    numIndicesLines= sizeof(indices_lines) / sizeof(*indices_lines);
    numIndicesTriangles= sizeof(indices_triangles) / sizeof(*indices_triangles);

    groundIndicesLines = new ResizingArray<GLushort>(numIndicesLines);
    groundIndicesTriangles = new ResizingArray<GLushort>(numIndicesTriangles);

    for(int i=0; i<numIndicesLines; i++) {
        groundIndicesLines->add(indices_lines[i]);
    }
    for(int i=0; i<numIndicesTriangles; i++) {
        groundIndicesTriangles->add(indices_triangles[i]);
    }

    groundColors = new ResizingArray<GLfloat>(componentCount * 3);
    for(int i=0; i<componentCount; i++) {
        groundColors->add(1.0); //0
        groundColors->add(1.0);
        groundColors->add(1.0);
    }
}

void sceneManager::setupParticleData() {
    if (drawFlags & PARTICLES) {
        // 3 components per vertex, 2 vertices per particle (particle is a little line segment)
        particleComponents = new ResizingArray<GLfloat>(3 * 2 * particleCount);

        particleIndices = new ResizingArray<GLushort>(2 * particleCount);
        for(int i = 0; i < 2 * particleCount; i++) {
            particleIndices->add(i);
        }

        // 4 floats per color(rgba), 2 colors per particle (each end of line seg.)
        particleColors = new ResizingArray<GLfloat>(4 * 2 * particleCount);

//        for(int j=0; j<particleCount; j++) {
//            // line segment starts at red..
//            particleColors->add(1.0); // r
//            particleColors->add(0.0); // g
//            particleColors->add(0.0); // b
//
//            // ..to white
//            particleColors->add(1.0); // r
//            particleColors->add(1.0); // g
//            particleColors->add(1.0); // b
//        }
        // component data & color data updated in draw loop
    }
}

sceneManager::~sceneManager() {
    delete volcanoComponents;
    delete volcanoColors;
    delete volcanoIndices;

    delete groundComponents;
    delete groundColors;
    delete groundIndicesLines;
    delete groundIndicesTriangles;

    if (drawFlags & TEXTURE) {
        glDeleteTextures(1, &textureUnit0);
    }

    glDeleteProgram(mProgram_particles);
    glDeleteProgram(mProgram_texmesh);

}

//// set param m to identity matrix.  assumes 4x4 matrix in column major order (OpenGL convention)
//// todo: delete, dead code
//void sceneManager::setIdentity(float* m) {
//    m[0] = m[5] = m[10] = m[15] = 1;
//    m[1] = m[2] = m[3] = m[4] = m[6] = m[7] = m[8] = m[9] = m[11]= m[12]= m[13]= m[14]= 0;
//}

// replacement for Matrix.setLookAtM()
// todo: use this in place of setLookAtM() to see if program still works.  if true, then use same code
// on native side instead of hardcoded chunk in changeLookAt()
// ported to java from Songho's code: http://songho.ca/opengl/gl_camera.html#lookat
void sceneManager::setLookAt(Matrix3x3& matrix,		// in/out
               const Vec3& eye,                     // in
               const Vec3& target,                  // in
               const Vec3& upDir) const             // in
{
    Vec3 forward = Vec3::subtract(eye, target);
    forward.normalize();

    Vec3 left = Vec3::cross(upDir, forward);
    left.normalize();

    Vec3 up = Vec3::cross(forward, left);

    matrix.makeIdentity();

    // set rotation part, inverse rotation matrix: M^-1 = M^T for Euclidean transform
    matrix.set(0, left.getx());         //    matrix[0] = left.a;
    matrix.set(4, left.gety());         //    matrix[4] = left.b;
    matrix.set(8, left.getz());         //    matrix[8] = left.c;
    matrix.set(1, up.getx());           //    matrix[1] = up.a;
    matrix.set(5, up.gety());           //    matrix[5] = up.b;
    matrix.set(9, up.getz());           //    matrix[9] = up.c;
    matrix.set(2, forward.getx());      //    matrix[2] = forward.a;
    matrix.set(6, forward.gety());      //    matrix[6] = forward.b;
    matrix.set(10, forward.getz());     //    matrix[10]= forward.c;

    // set translation part
    matrix.set(12, -left.getx() * eye.getx() - left.gety() * eye.gety() - left.getz() * eye.getz());      //    matrix[12]= -left.a * eye.a - left.b * eye.getb - left.c * eye.c;
    matrix.set(13, -up.getx() * eye.getx() - up.gety() * eye.gety() - up.getz() * eye.getz());      //    matrix[13]= -up.a * eye.a - up.b * eye.getb - up.c * eye.getc;
    matrix.set(14, -forward.getx() * eye.getx() - forward.gety() * eye.gety() - forward.getz() * eye.getz());      //    matrix[14]= -forward.a * eye.a - forward.b * eye.b - forward.c * eye.c;
}

void sceneManager::changeLookAt() {
    Vec3 cameraPos;
    Vec3 upVector;
    Matrix3x3 mLookAtMat;

    cameraPos.putx(vCameraPos[0]);
    cameraPos.puty(vCameraPos[1]);
    cameraPos.putz(vCameraPos[2]);

    upVector.set(vUpVector[0], vUpVector[1], vUpVector[2]);

//    muMVPMatrixHandle = glGetUniformLocation(mProgram_particles, "uMVPMatrix");
    Vec3 target(0,0,0);

    // old look at matrix was hardcoded, now use handbuilt setLookAt()
    setLookAt(mLookAtMat, cameraPos, target, upVector);

    mLookAtMat.debugPrint(debugLog, "mLookAtMat");
    mOrientation.debugPrint(debugLog, "mOrientation");

    Matrix3x3::mul(mVMatrix, mLookAtMat, mOrientation);

    // mVMatrix is combined with projection matrix in drawframe()
    mVMatrix.debugPrint(debugLog, "mVMatrix");
}

void sceneManager::toggleTouchdown() {
    // cycle draw modes
    //drawFlags; WIREFRAME, TEXTURE, <neither>
    // INITIAL: drawFlags = OBELISK | PARTICLES | GROUND; // controls what objects are drawn (obelisk, particles, textures, etc.)
    current_mode == mode_count-1 ? current_mode = 0 : current_mode++;
    drawFlags = drawModes[current_mode];

    changeLookAt();
}

void sceneManager::storeOrientation(float* in) {
    mOrientation.put(in);
}

void sceneManager::printGLString(const char *name, GLenum s) {
    const char *v = (const char *) glGetString(s);
    LOGI("GL %s = %s\n", name, v);
}

void sceneManager::checkGlError(const char* op) {
    for (GLint error = glGetError(); error; error = glGetError()) {
        LOGI("after %s() glError (0x%x)\n", op, error);
    }
}

GLuint sceneManager::loadShader(GLenum shaderType, const char* pSource) {
    GLuint shader = glCreateShader(shaderType);
    if (shader) {
        glShaderSource(shader, 1, &pSource, NULL);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (!compiled) {
            GLint infoLen = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
            if (infoLen) {
                char* buf = new char[infoLen];
                if (buf) {
                    glGetShaderInfoLog(shader, infoLen, NULL, buf);
                    LOGE("Could not compile shader %d:\n%s\n",
                         shaderType, buf);
                    delete [] buf;
                }
                glDeleteShader(shader);
                shader = 0;
            }
        }
    }
    return shader;
}

void sceneManager::createPrograms(const char* pVertexSource, const char* pFragmentSource,
                                  const char* pVertexTextureSrc, const char* pFragmentTextureSrc)
{
    createProgram(&mProgram_particles, pVertexSource, pFragmentSource);
    createProgram(&mProgram_texmesh, pVertexTextureSrc, pFragmentTextureSrc);
}

// create OpenGL program given source to vertex and fragment shaders
// pVertexSource[in]- pointer to vertex program src
// pFragmentSource[in]- pointer to fragment program src
// program [in/out]- pointer to program identifier, stored as member
void sceneManager::createProgram(GLuint* program, const char* pVertexSource, const char* pFragmentSource) {
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER, pVertexSource);
    if (!vertexShader) {
        // todo: handle error
        return;
    }

    GLuint pixelShader = loadShader(GL_FRAGMENT_SHADER, pFragmentSource);
    if (!pixelShader) {
        // todo: handle error
        return;
    }

    /*gProgram*/ *program = glCreateProgram();
    if (*program) {
        glAttachShader(*program, vertexShader);
        checkGlError("glAttachShader");
        glAttachShader(*program, pixelShader);
        checkGlError("glAttachShader");
        glLinkProgram(*program);
        GLint linkStatus = GL_FALSE;
        glGetProgramiv(*program, GL_LINK_STATUS, &linkStatus);
        if (linkStatus != GL_TRUE) {
            GLint bufLength = 0;
            glGetProgramiv(*program, GL_INFO_LOG_LENGTH, &bufLength);
            if (bufLength) {

                char* buf = new char[bufLength];
                if (buf) {
                    glGetProgramInfoLog(*program, bufLength, NULL, buf);
                    LOGE("Could not link program:\n%s\n", buf);
                    delete[] buf;
                }
            }
            glDeleteProgram(*program);
            *program = 0;
        }
    }

    if (!*program) {
        LOGE("Could not create program.");
        return;
    }
}

void sceneManager::surfaceChanged(int w, int h) {
    // only do this once, then it won't be called again when the user rotates their device with the rotation set to 'unlocked'
    if (!windowInitialized)
    {
        glViewport(0, 0, w, h);

        if (h>w) {
            vUpVector[0] = 0.0f;
            vUpVector[1] = 1.0f;
            vUpVector[2] = 0.0f;
        }
        else {
            vUpVector[0] = 1.0f;
            vUpVector[1] = 0.0f;
            vUpVector[2] = 0.0f;
        }

        Matrix3x3::frustum(mProjMatrix, -1, 1, -1, 1, 1, 8); // make projection matrix given clip planes

        changeLookAt();
        windowInitialized = true;
    }
}

// set program state for drawing with textures
void sceneManager::setTextureProgram() {
    glUseProgram(mProgram_texmesh);
    // send the composite modelview projection matrix to the vertex shader
    muMVPMatrixHandle = glGetUniformLocation(mProgram_texmesh, "uMVPMatrix");
    glUniformMatrix4fv(muMVPMatrixHandle, 1, false, mMVPMatrix.get());
}

// set state for drawing basic vertex/fragment shader pair (flat shading or wireframe)
void sceneManager::setBasicProgram() {
    glUseProgram(mProgram_particles);
    muMVPMatrixHandle = glGetUniformLocation(mProgram_particles, "uMVPMatrix");
    glUniformMatrix4fv(muMVPMatrixHandle, 1, false, mMVPMatrix.get());
}

void sceneManager::drawFrame() {
    long timeSincelastupdate = now_ms()/*SystemClock.elapsedRealtime()*/
                               - updatetime;
    // 24 times per second, update the particles
    if (drawFlags & PARTICLES && timeSincelastupdate > 1000 / updates_second) // 1000ms/24 = 42
    {
        {    // decrement lifetime for all live particles
            std::vector<AParticle>::iterator it = particles.begin();
            while (it != particles.end()) {
                if ((*it).get_lifeleft() > 0) {
                    (*it).subtract_lifetime(1000 / updates_second);
                }
                it++;
            }
        }

        // remove particles that are past their lifetime.
        // currently only removes the first expired particle encountered in the list
        // this should be ok considering the frequency the list is checked
        for (int i = 0; i < particles.size(); i++) {
            if (particles[i].expired()) {
//                    particles.remove(i);
                if (i < particles.size()) {
                    // remove item at position i
                    std::vector<AParticle>::iterator it;
                    it = particles.begin();
                    int position = 1;
                    while (it != particles.end()) {
                        if (position == i)
                            break;
                        it++;
                    }
                    particles.erase(it);
                    // end remove
                }
                break;
            }
        }

        // Generate particles
        if (now_ms() - lastParticleTime >
            1000 / particles_per_sec) // 1 particle/sec
        {
            // speed of particle upper and lower bounds
            float lower = 0.08f; //.7
            float upper = 0.092f; //.12
            float rnd1 = rnd_float();
            LOGI("random #1: %f()\n", rnd1);
            float magnitude = rnd1 * (upper - lower) + lower;

            // given a random angle within range alpha_s, and magnitude (speed), calculate y, z coordinates
            float alpha_s = toRadians(30); // angle spread from xy plane
            float half_alpha_s = alpha_s / 2.0f;
            float rnd2 = rnd_float();
            LOGI("random #2: %f()\n", rnd2);
            float angle_a = rnd2 * alpha_s;
            float right_angle = toRadians(90);

            //LOGI("\t%.3f\t%.3f\t%.3f\t%.3f", m[0], m[4], m[8], m[12]);
            LOGI("\tright_angle: %.3f", right_angle);
            LOGI("\thalf_alpha_s: %.3f", half_alpha_s);
            LOGI("\tangle_a: %.3f", angle_a);

            Vec3 particle_vector;
            particle_vector.set(0.0, 0.0, 0.0);
            particle_vector.setd(1.0);

            // magnitude & angle_a are random values
            particle_vector.setb(magnitude * (float) cos(right_angle - half_alpha_s + angle_a));
            particle_vector.setc(magnitude * (float) sin(right_angle - half_alpha_s + angle_a));

            float rotateYaxisArray[16];

            float beta_s = toRadians(30); // angle spread from yz plane
            float aay_radians = beta_s * rnd_float() - beta_s / 2.0f;

//            LOGI("cos(aay_rad): %f", resultVec.geta());
//            LOGI("sin(aay_rad): %f", resultVec.geta());

            // setup rotation matrix for rotating about y axis by angle_about_y
            rotateYaxisArray[0] = static_cast<float>(cos(aay_radians));            // column 0
            rotateYaxisArray[1] = 0;
            rotateYaxisArray[2] = static_cast<float>(-sin(aay_radians));
            rotateYaxisArray[3] = 0;

            rotateYaxisArray[4] = 0;                                        // column 1
            rotateYaxisArray[5] = 1;
            rotateYaxisArray[6] = 0;
            rotateYaxisArray[7] = 0;

            rotateYaxisArray[8] = static_cast<float>(sin(aay_radians));            // column 2
            rotateYaxisArray[9] = 0;
            rotateYaxisArray[10] = static_cast<float>(cos(aay_radians));
            rotateYaxisArray[11] = 0;

            rotateYaxisArray[12] = 0;                                        // column 4
            rotateYaxisArray[13] = 0;
            rotateYaxisArray[14] = 0;
            rotateYaxisArray[15] = 1;

            Vec3 resultVec;

            // result = lhs x rhs --- resultVec(vec) = rotateYaxisArray(mat) x particle_vector(vec)
            Matrix3x3 rotateYaxisMatrix(rotateYaxisArray);

            rotateYaxisMatrix.debugPrint(true, "rotateYaxisMatrix");
            particle_vector.debugPrint(true, "particle_vector");
            Matrix3x3::mul(resultVec, rotateYaxisMatrix, particle_vector);
            resultVec.debugPrint(true, "resultVec");

            particle_vector.seta(resultVec.geta());
            particle_vector.setb(resultVec.getb());
            particle_vector.setc(resultVec.getc());
            particle_vector.setd(resultVec.getd());

            LOGI("particle_vector[0] %.3f", particle_vector.geta());
            LOGI("particle_vector[1] %.3f", particle_vector.getb());
            LOGI("particle_vector[2] %.3f", particle_vector.getc());
            LOGI("particle_vector[3] %.3f", particle_vector.getd());

            AParticle newparticle(0.0f, 0.0f, 1.0f, // location of emitter, a point source
                                  particle_vector.geta(), // velocity and direction of particle
                                  particle_vector.getb(),
                                  particle_vector.getc(),
                                  1.0f, 0.0f, 0.0f, 1.0f, // initial color of particle
                                  1.0f, 1.0f, 1.0f, 1.0f, // final color of particle
                                  gravity,
                                  2000); // static particle life of 2000 ms.  could randomize?
            particles.push_back(newparticle);
            lastParticleTime = now_ms();
        }

        // update position of all particles
        for (int i = 0; i < particles.size(); i++) {
            particles[i].UpdateAttributes();
            particles[i].UpdatePosition();
            particles[i].applyGravity();
        }

        // refresh time of last update
        updatetime = now_ms();

        // copy the particles into the floatbuffer containing the vertices
//            vertices.position(vertexdata.length); // set position to the end of the floatbuffer- why is this done
        // particleColors.position(4); -- why is position set to 4 here?

        // 2 vertices per particle, 3 components per vertex

        // 3 floats per particle (x,y,z), 2 points(start, end) for each line segment
        // todo: check if vertices are initialized to zero on allocation
        //vertices = new float[2 * 3 * (particles.size() + obeliskVerts)];
        int vPos = 0;
        int pcPos = 0;

        // todo: check if allocation initializes to 0
        //indicies = new GLushort[2 * 3 * particles.size() + obeliskVerts];

        std::vector<AParticle>::iterator it = particles.begin();
        for (; it != particles.end(); it++) { // std::vector<AParticle> particles;
            Point point = (*it).getRenderPos();
            particleComponents->assign(vPos++, point.getx());
            particleComponents->assign(vPos++, point.gety());
            particleComponents->assign(vPos++, point.getz());

            // Log.d(TAG, "red: " + particle.getr());
            particleColors->assign(pcPos++, (*it).getr());

            // Log.d(TAG, "grn: " + particle.getg());
            particleColors->assign(pcPos++, (*it).getg());

            // Log.d(TAG, "blu: " + particle.getb());
            particleColors->assign(pcPos++, (*it).getb());

            // Log.d(TAG, "alpha: " + particle.geta());
            particleColors->assign(pcPos++, (*it).geta());

            Point prev = (*it).getPreviousPos(); // trailing part of line segment
            particleComponents->assign(vPos++, prev.getx());
            particleComponents->assign(vPos++, prev.gety());
            particleComponents->assign(vPos++, prev.getz());

            // Log.d(TAG, "red: " + particle.getr());
            particleColors->assign(pcPos++, (*it).getr());

            // Log.d(TAG, "grn: " + particle.getg());
            particleColors->assign(pcPos++, (*it).getg());

            // Log.d(TAG, "blu: " + particle.getb());
            particleColors->assign(pcPos++, (*it).getb());

            // Log.d(TAG, "alpha: " + particle.geta());
            particleColors->assign(pcPos++, (*it).geta());
        }
    }

    changeLookAt();

    mProjMatrix.debugPrint(debugLog, "mProjMatrix");
    mVMatrix.debugPrint(debugLog, "mVMatrix");
    Matrix3x3::mul(mMVPMatrix, mProjMatrix, mVMatrix); // result, lhs, rhs (result = lhs x rhs)

    mMVPMatrix.debugPrint(debugLog, "mMVPMatrix");

    // Redraw background color
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (drawFlags & TEXTURE) {
        setBasicProgram();
        drawParticles();

        setTextureProgram();
        drawGroundWithTexture();

        setBasicProgram();
        drawVolcano();
    }
    else {
        setBasicProgram();
        drawParticles();
        drawGroundNoTexture();
        drawVolcano();
    }

    framesdrawn++;
    // every few seconds, write average fps over the last 10 seconds to log
    if (now_ms()-intervalbegin > 4000)// velocity and direction of particle
    {
        long elapsed = now_ms() - intervalbegin;
        double fps = static_cast<double>(framesdrawn) / static_cast<double>(elapsed/1000);
//            Log.d(TAG, "fps: " + fps);
        // ALOG("This message comes from C at line %d.", __LINE__);
        LOGI("fps: %f", fps);
        intervalbegin = now_ms();
        framesdrawn = 0;
    }
}

void sceneManager::drawVolcano() {
    if (OBELISK & drawFlags) {
        glVertexAttribPointer(vertex_attrib_idx,
                              3, // # of components per vertex attribute. Must be 1, 2, 3, or 4.
                              GL_FLOAT,
                              GL_FALSE, // Normalized?
                              3 * sizeof(GLfloat), // byte offset between vertex attributes. attribute is a set of elements
                              volcanoComponents->data()); // define vertex array

        GLfloat* colorData = volcanoColors->data();
        glVertexAttribPointer(color_attrib_idx,
                              3, // # of components per generic vertex attribute
                              GL_FLOAT,
                              GL_FALSE, // normalized?
                              3* sizeof(GLfloat), // byte offset between attributes. attribute is a set of elements
                              colorData);

        glEnableVertexAttribArray(vertex_attrib_idx);
        glEnableVertexAttribArray(color_attrib_idx);

        // draw triangles for volcano
        glDrawElements(GL_TRIANGLES,
                       volcanoIndices->size(),    // # of indicies in index array (# of short values, last param)
                       GL_UNSIGNED_SHORT ,  // data type of index array
                       volcanoIndices->data());   // indicies_array

        glDisableVertexAttribArray(vertex_attrib_idx);
        glDisableVertexAttribArray(color_attrib_idx);
    }
}

void sceneManager::drawParticles() {
    if (drawFlags & PARTICLES) {
        vertex_attrib_idx = glGetAttribLocation(mProgram_particles, "vPosition");
        glVertexAttribPointer(vertex_attrib_idx,
                              3, // # of components per vertex attribute. Must be 1, 2, 3, or 4.
                              GL_FLOAT,
                              GL_FALSE, // Normalized?
                              3 * sizeof(GLfloat), // byte offset between vertex attributes. attribute is a set of elements
                              particleComponents->data()); // define vertex array

        GLfloat* colorData = particleColors->data();
        color_attrib_idx = glGetAttribLocation(mProgram_particles, "vColor");
        glVertexAttribPointer(color_attrib_idx,
                              3, // # of components per generic vertex attribute
                              GL_FLOAT,
                              GL_FALSE, // normalized?
                              3* sizeof(GLfloat), // byte offset between attributes. attribute is a set of elements
                              colorData);

        glEnableVertexAttribArray(vertex_attrib_idx);
        glEnableVertexAttribArray(color_attrib_idx);

        // Draw the particles.  each particle is a little line segment
        glDrawElements(GL_LINES,
                       particleIndices->size(),    // # of indicies in index array (# of short values, last param)
                       GL_UNSIGNED_SHORT ,  // data type of index array
                       particleIndices->data());   // indicies_array

        glDisableVertexAttribArray(vertex_attrib_idx);
        glDisableVertexAttribArray(color_attrib_idx);
    }
}

void sceneManager::drawGroundNoTexture() {
    if ( flagSet(drawFlags, GROUND) && !flagSet(drawFlags, TEXTURE) ) {
        vertex_attrib_idx = glGetAttribLocation(mProgram_particles, "vPosition");
        glVertexAttribPointer(static_cast<GLuint>(vertex_attrib_idx),
                              3, // # of components per vertex attribute. Must be 1, 2, 3, or 4.
                              GL_FLOAT, // data type for component
                              GL_FALSE, // Normalized?
                              3 * sizeof(GLfloat), // byte offset between vertex attributes. attribute is a set of elements
                              groundComponents->data()); // define vertex array

        GLfloat* colorData = groundColors->data();
        color_attrib_idx = glGetAttribLocation(mProgram_particles, "vColor");
        glVertexAttribPointer(static_cast<GLuint>(color_attrib_idx),
                              3, // # of components per generic vertex attribute
                              GL_FLOAT,
                              GL_FALSE, // normalized?
                              3 * sizeof(GLfloat), // byte offset between attributes. attribute is a set of elements
                              colorData);

        glEnableVertexAttribArray(vertex_attrib_idx);
        glEnableVertexAttribArray(color_attrib_idx);

        if (drawFlags & WIREFRAME) {
            glDrawElements(GL_LINES,
                           groundIndicesLines->size(),    // # of indicies in index array (# of short values, last param)
                           GL_UNSIGNED_SHORT,             // data type of index array
                           groundIndicesLines->data());   // indicies_array
        }
        else {
            glDrawElements(GL_TRIANGLES,
                           groundIndicesTriangles->size(),    // # of indicies in index array (# of short values, last param)
                           GL_UNSIGNED_SHORT,             // data type of index array
                           groundIndicesTriangles->data());   // indicies_array
        }

        glDisableVertexAttribArray(vertex_attrib_idx);
        glDisableVertexAttribArray(color_attrib_idx);
    }
}

void sceneManager::drawGroundWithTexture() {
    if (flagSet(drawFlags, GROUND | TEXTURE)) {
        vertex_attrib_idx = glGetAttribLocation(mProgram_texmesh, "vPosition");
        glVertexAttribPointer(static_cast<GLuint>(vertex_attrib_idx),
                              3, // # of components per vertex attribute. Must be 1, 2, 3, or 4.
                              GL_FLOAT, // data type for component
                              GL_FALSE, // Normalized?
                              3 * sizeof(GLfloat), // byte offset between vertex attributes. attribute is a set of elements
                              groundComponents->data()); // define vertex array

        texture_coords_idx = glGetAttribLocation(mProgram_texmesh, "vTexCoords");
        glVertexAttribPointer(static_cast<GLuint>(texture_coords_idx),
                              2, // # of components per generic vertex attribute
                              GL_FLOAT,
                              GL_FALSE, // normalized?
                              2 * sizeof(GLfloat), // byte offset between attributes. attribute is a set of elements
                              groundTexComponents->data());

        glEnableVertexAttribArray(vertex_attrib_idx);
        glEnableVertexAttribArray(texture_coords_idx);

        if (drawFlags & WIREFRAME) {
            glDrawElements(GL_LINES,
                           groundIndicesLines->size(),    // # of indicies in index array (# of short values, last param)
                           GL_UNSIGNED_SHORT,             // data type of index array
                           groundIndicesLines->data());   // indicies_array
        }
        else {
            glDrawElements(GL_TRIANGLES,
                           groundIndicesTriangles->size(),    // # of indicies in index array (# of short values, last param)
                           GL_UNSIGNED_SHORT,             // data type of index array
                           groundIndicesTriangles->data());   // indicies_array
        }

        glDisableVertexAttribArray(vertex_attrib_idx);
        glDisableVertexAttribArray(texture_coords_idx);
    }
}

static sceneManager masterScene;

jint JNI_OnLoad(JavaVM *vm, void *reserved) {
    LOGI ("JNI_OnLoad");
    return JNI_VERSION_1_6;
}

// good place to create textures, or pass in from java side (resource -> byte array -> void* ?)
void initialize(const char* strVertexSrc, const char* strFragmentSrc,
                const char* strTexVpSrc, const char* strTexFpSrc)
{
	masterScene.printGLString("Version", GL_VERSION);
    masterScene.printGLString("Vendor", GL_VENDOR);
    masterScene.printGLString("Renderer", GL_RENDERER);
    masterScene.printGLString("Extensions", GL_EXTENSIONS);

//	LOGI("setupGraphics(%d, %d)", w, h);
    masterScene.createPrograms(strVertexSrc, strFragmentSrc,
                               strTexVpSrc, strTexFpSrc);
    masterScene.initTextures();
}

void drawframe(int invalue) {
    masterScene.drawFrame();
}

// device orientation matrix is passed into native side from device
// device -> java side -> JNI -> native side
void receiveMatrix(float* matIn) {
    masterScene.storeOrientation(matIn);
}

extern "C" {
JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_surfaceChanged
        (JNIEnv *, jclass, jint inwidth, jint inheight) {

}

JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1surfaceChanged
        (JNIEnv *, jclass, jint inwidth, jint inheight) {
    masterScene.surfaceChanged(inwidth, inheight);
}

JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1touchdownHandler
        (JNIEnv *, jclass, jint invalue) {
    masterScene.toggleTouchdown();
}
/*
 * Class:     com_neusoft_particle_ObjectJNI
 * Method:    jni_initialize
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
// public static native void jni_initialize(String strVertexSrc, String strFragmentSrc);

//public static native void jni_initialize(String strVertexSrc, String strFragmentSrc);
JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1initialize
        (JNIEnv *env, jclass, jstring strVertexSrc, jstring strFragmentSrc,
        jstring strTextureVp, jstring strTextureFp)
{
    unsigned char whatisthis = 0;
    const char *strVertex = env->GetStringUTFChars(strVertexSrc, &whatisthis);
    const char *strFragment = env->GetStringUTFChars(strFragmentSrc, &whatisthis);

    const char *strTextureVertex = env->GetStringUTFChars(strTextureVp, &whatisthis);
    const char *strTextureFragment = env->GetStringUTFChars(strTextureFp, &whatisthis);

    initialize(strVertex, strFragment, strTextureVertex, strTextureFragment);
}

/*
 * Class:     com_neusoft_particle_ObjectJNI
 * Method:    jni_drawframe
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1drawframe
        (JNIEnv *, jclass, jint invalue) {
    drawframe(invalue);
}

JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1receiveMatrix
        (JNIEnv *env, jclass, jfloatArray jfa) {
    jfloatArray result;
    result = env->NewFloatArray(16);
    if (result == NULL) {
        // todo: handle error, can't allocate new float array
    }

    jfloat matrix[16];
    jfloat *pmatrix = env->GetFloatArrayElements(jfa, 0);

    receiveMatrix(pmatrix);

    env->ReleaseFloatArrayElements(jfa, pmatrix, 0);
}
} // end extern "C"


