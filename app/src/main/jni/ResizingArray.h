//
// Created by Steve on 3/10/2017.
//

#ifndef RESIZINGARRAY_H
#define RESIZINGARRAY_H

template<typename T>
class ResizingArray {
    int N; // N is # of items, as per convention in Algorithms, 4th ed. Sedgewick, Wayne
    int length; // the length of the array
    T* items;

    const static int default_size = 100;
//    std::vector<T> vItems;
public:
    ResizingArray() {
        N = 0;
        items = 0;
        items = new T[default_size]();
    }

    ResizingArray(int size) {
        //items = new T[N];
        N = 0;
        items = 0;
        items = new T[size];
        length = size;
    }

    ~ResizingArray() {
        if (items) {
            delete[] items;
        }
    }

    // todo: copy constructor, copy assignment
    ResizingArray<T>& operator=(const ResizingArray& rhs) {
        // check for assignment to self
        if (this == &rhs)
            return *this;

        resize(rhs.size());

        memcpy(items, rhs.items, sizeof(T) * rhs.size());
        N = rhs.N;
        length = rhs.length;

        return *this;
    }

    bool isEmpty() const {
        return N == 0;
    }

    int size() const {
        return N;
    }

    void resize(int newsize) {
        T* tempItems = new T[newsize](); // (T*)malloc(newsize*sizeof(T));
        for(int i=0; i<N; i++) {
            tempItems[i] = items[i];
        }
        delete[] items;
        items = tempItems;
        tempItems = 0;
    }

    // add element to end of list (aka push)
    void add(float newElement) {
        if (N == length) {
            resize(2 * length);
        }
        items[N++] = newElement;
    }

    // remove a float from end of list (aka pop).  resize array if needed
    T remove() { // Remove item from top of stack.
        T item = items[--N];

        if (N > 0 && N == length / 4) {
            resize(length / 2);
        }
        return item;
    }

    //void put

    // todo: implement overwrite val at index. figure out resize and index bound logic
    void assign(int index, T newValue) {
        if (index > length-1) {
            // todo: resize to bigger, or throw error?
        }
        else {
            // else if (index > N). If index larger than largest, but smaller than size of buffer, still assign
            items[index] = newValue;
        }
    }

    T* data() {
        return items;
    }
};

#endif //RESIZINGARRAY_H