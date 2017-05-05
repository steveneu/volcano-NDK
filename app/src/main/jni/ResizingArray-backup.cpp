//
// Created by Steve on 3/10/2017.
//

#ifndef PARTICLE_PORT_STUDIO_RESIZINGARRAY_CPP
#define PARTICLE_PORT_STUDIO_RESIZINGARRAY_CPP

#include <GLES2/gl2.h>

//template<typename T>
class ResizingArray {
    int N; // N is # of items, as per convention in Algorithms, 4th ed. Sedgewick, Wayne
    GLfloat* items;
public:
    ResizingArray() {
        N = 0;
    }

    ResizingArray(int size) {
        N = size;
        items = new GLfloat[N];
    }

    ~ResizingArray() {
        delete [] items;
    }

    // todo: copy constructor, copy assignment
    ResizingArray& operator=(const ResizingArray& rhs) {
        int elements = sizeof(rhs.items)/sizeof(*rhs.items);
        this->items = new GLfloat[elements];

        memcpy(this->items, rhs.items, elements * sizeof(GLfloat));

        return *this;
    }

    void assign(int index, GLfloat element) {
        int elements = sizeof(items)/sizeof(*items);
        if (N == elements) {
            resize(2*elements);
        }
        items[N++] = element;
    }

    bool isEmpty() {
        return N == 0;
    }

    int size() {
        return N;
    }

    // add element to end of list (aka push)
    void add(float element) {
        int elements = sizeof(items)/sizeof(*items);
        if (N == elements) {
            resize(2*elements);
        }

        items[N++] = element;
    }

    // remove a float from end of list (aka pop).  resize array if needed
    GLfloat remove()
    { // Remove item from top of stack.
        GLfloat item = items[--N];
        items[N] = 0;
        //a[N] = null; // Avoid loitering (see text).
        int elements = sizeof(items)/sizeof(*items);

        if (N > 0 && N == elements/4)
            resize(elements/2);
        return item;
    }

    GLfloat* data() {
        return items;
    }

    void resize(int max) {
        GLfloat* temp = new GLfloat[max];
        for(int i=0; i<N; i++) {
            temp[i] = items[i];
        }

        delete [] items;
        items = temp;
    }
};

#endif //PARTICLE_PORT_STUDIO_RESIZINGARRAY_CPP
