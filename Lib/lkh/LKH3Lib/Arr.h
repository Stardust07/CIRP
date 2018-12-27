////////////////////////////////
/// usage : 1.	fixed size 1D and 2D array following RAII.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_LKH3LIB_ARR_H
#define SMART_SZX_GOAL_LKH3LIB_ARR_H


#include <algorithm>
#include <initializer_list>
#include <vector>

#include <cstring>


namespace szx {

#ifndef SMART_SZX_CPP_UTILIBS_ARR
#define SMART_SZX_CPP_UTILIBS_ARR
template<typename T, typename IndexType = int>
class Arr {
public:
    // it is always valid before copy assignment due to no reallocation.
    using Iterator = T * ;
    using ConstIterator = T const *;

    enum ResetOption { AllBits0 = 0, AllBits1 = -1 };

    explicit Arr() : arr(nullptr), len(0) {}
    explicit Arr(IndexType length) { allocate(length); }
    explicit Arr(IndexType length, T *data) : arr(data), len(length) {} // make sure `this` will be the only one who owns `data`.
    explicit Arr(IndexType length, const T &defaultValue) : Arr(length) {
        std::fill(arr, arr + length, defaultValue);
    }
    explicit Arr(std::initializer_list<T> l) : Arr(static_cast<IndexType>(l.size())) {
        std::copy(l.begin(), l.end(), arr);
    }

    Arr(const Arr &a) : Arr(a.len) {
        if (this != &a) { copyData(a.arr); }
    }
    Arr(Arr &&a) : Arr(a.len, a.arr) { a.arr = nullptr; }

    Arr& operator=(const Arr &a) {
        if (this != &a) {
            if (len != a.len) {
                clear();
                init(a.len);
            }
            copyData(a.arr);
        }
        return *this;
    }
    Arr& operator=(Arr &&a) {
        if (this != &a) {
            delete[] arr;
            arr = a.arr;
            len = a.len;
            a.arr = nullptr;
        }
        return *this;
    }

    ~Arr() { clear(); }

    // allocate memory if it has not been init before.
    bool init(IndexType length) {
        if (arr == nullptr) { // avoid re-init and memory leak.
            allocate(length);
            return true;
        }
        return false;
    }

    // remove all items.
    void clear() {
        delete[] arr;
        arr = nullptr;
    }

    // set all data to val. any value other than 0 or -1 is undefined behavior.
    void reset(ResetOption val = ResetOption::AllBits0) { memset(arr, val, sizeof(T) * len); }

    T& operator[](IndexType i) { return arr[i]; }
    const T& operator[](IndexType i) const { return arr[i]; }

    T& at(IndexType i) { return arr[i]; }
    const T& at(IndexType i) const { return arr[i]; }

    Iterator begin() { return arr; }
    Iterator end() { return (arr + len); }
    ConstIterator begin() const { return arr; }
    ConstIterator end() const { return (arr + len); }
    T& front() { return at(0); }
    T& back() { return at(len - 1); }
    const T& front() const { return at(0); }
    const T& back() const { return at(len - 1); }

    IndexType size() const { return len; }
    bool empty() const { return (len == 0); }

protected:
    // must not be called except init.
    void allocate(IndexType length) {
        // TODO[szx][2]: length > (1 << 32)?
        arr = new T[static_cast<size_t>(length)];
        len = length;
    }

    void copyData(T *data) {
        // TODO[szx][1]: what if data is shorter than arr?
        // OPTIMIZE[szx][8]: use memcpy() if all callers are POD type.
        std::copy(data, data + len, arr);
    }


    T *arr;
    IndexType len;
};


template<typename T, typename IndexType = int>
class Arr2D {
public:
    // it is always valid before copy assignment due to no reallocation.
    using Iterator = T * ;
    using ConstIterator = T const *;

    enum ResetOption { AllBits0 = 0, AllBits1 = -1 };

    explicit Arr2D() : arr(nullptr), len1(0), len2(0), len(0) {}
    explicit Arr2D(IndexType length1, IndexType length2) { allocate(length1, length2); }
    explicit Arr2D(IndexType length1, IndexType length2, T *data)
        : arr(data), len1(length1), len2(length2), len(length1 * length2) {
    }
    explicit Arr2D(IndexType length1, IndexType length2, const T &defaultValue) : Arr2D(length1, length2) {
        std::fill(arr, arr + len, defaultValue);
    }

    Arr2D(const Arr2D &a) : Arr2D(a.len1, a.len2) {
        if (this != &a) { copyData(a.arr); }
    }
    Arr2D(Arr2D &&a) : Arr2D(a.len1, a.len2, a.arr) { a.arr = nullptr; }

    Arr2D& operator=(const Arr2D &a) {
        if (this != &a) {
            if (len != a.len) {
                clear();
                init(a.len1, a.len2);
            } else {
                len1 = a.len1;
                len2 = a.len2;
            }
            copyData(a.arr);
        }
        return *this;
    }
    Arr2D& operator=(Arr2D &&a) {
        if (this != &a) {
            delete[] arr;
            arr = a.arr;
            len1 = a.len1;
            len2 = a.len2;
            len = a.len;
            a.arr = nullptr;
        }
        return *this;
    }

    ~Arr2D() { clear(); }

    // allocate memory if it has not been init before.
    bool init(IndexType length1, IndexType length2) {
        if (arr == nullptr) { // avoid re-init and memory leak.
            allocate(length1, length2);
            return true;
        }
        return false;
    }

    // remove all items.
    void clear() {
        delete[] arr;
        arr = nullptr;
    }

    // set all data to val. any value other than 0 or -1 is undefined behavior.
    void reset(ResetOption val = ResetOption::AllBits0) { memset(arr, val, sizeof(T) * len); }

    IndexType getFlatIndex(IndexType i1, IndexType i2) const { return (i1 * len2 + i2); }

    T* operator[](IndexType i1) { return (arr + i1 * len2); }
    const T* operator[](IndexType i1) const { return (arr + i1 * len2); }

    T& at(IndexType i) { return arr[i]; }
    const T& at(IndexType i) const { return arr[i]; }
    T& at(IndexType i1, IndexType i2) { return arr[i1 * len2 + i2]; }
    const T& at(IndexType i1, IndexType i2) const { return arr[i1 * len2 + i2]; }

    Iterator begin() { return arr; }
    Iterator begin(IndexType i1) { return arr + (i1 * len2); }
    ConstIterator begin() const { return arr; }
    ConstIterator begin(IndexType i1) const { return arr + (i1 * len2); }

    Iterator end() { return (arr + len); }
    Iterator end(IndexType i1) { return arr + (i1 * len2) + len2; }
    ConstIterator end() const { return (arr + len); }
    ConstIterator end(IndexType i1) const { return arr + (i1 * len2) + len2; }

    T& front() { return at(0); }
    T& front(IndexType i1) { return at(i1, 0); }
    const T& front() const { return at(0); }
    const T& front(IndexType i1) const { return at(i1, 0); }

    T& back() { return at(len - 1); }
    T& back(IndexType i1) { return at(i1, len - 1); }
    const T& back() const { return at(len - 1); }
    const T& back(IndexType i1) const { return at(i1, len - 1); }

    IndexType size1() const { return len1; }
    IndexType size2() const { return len2; }
    IndexType size() const { return len; }
    bool empty() const { return (len == 0); }

protected:
    // must not be called except init.
    void allocate(IndexType length1, IndexType length2) {
        len1 = length1;
        len2 = length2;
        len = length1 * length2;
        arr = new T[static_cast<size_t>(len)];
    }

    void copyData(T *data) {
        // TODO[szx][1]: what if data is shorter than arr?
        // OPTIMIZE[szx][8]: use memcpy() if all callers are POD type.
        std::copy(data, data + len, arr);
    }


    T *arr;
    IndexType len1;
    IndexType len2;
    IndexType len;
};
#endif // !SMART_SZX_CPP_UTILIBS_ARR

}


#endif // SMART_SZX_GOAL_LKH__ARR_H
