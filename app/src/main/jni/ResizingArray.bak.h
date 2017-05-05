//
// Created by Steve on 3/10/2017.
//

#ifndef PARTICLE_PORT_STUDIO_RESIZINGARRAY_H
#define PARTICLE_PORT_STUDIO_RESIZINGARRAY_H

#include <GLES2/gl2.h>

//template<typename T>
class ResizingArray {
    int N; // N is # of items, as per convention in Algorithms, 4th ed. Sedgewick, Wayne
    GLfloat* items;
public:
    ResizingArray();
    ResizingArray(int size);
    ~ResizingArray();

    // todo: copy constructor, copy assignment
    ResizingArray& operator=(const ResizingArray& rhs);
    void assign(int index, GLfloat element);
    bool isEmpty();
    int size();

    // add element to end of list (aka push)
    void add(float element);

    // remove a float from end of list (aka pop).  resize array if needed
    GLfloat remove();
    GLfloat* data();
    void resize(int max);
};

#endif //PARTICLE_PORT_STUDIO_RESIZINGARRAY_H
