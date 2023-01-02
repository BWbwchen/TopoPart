#ifndef TENSOR_H_
#define TENSOR_H_

#include <vector>

#include "type.h"

namespace topart {

using std::vector;

template <class T>
class Tensor {
public:
    vector<T> v;

public:
    Tensor(intg size) { v.resize(size, 0); };
    Tensor(){};
    T &at(intg i);

    /* T operator+(const T &rhs); */
    /* T operator-(const T &rhs); */
    Tensor<T> &operator+=(const Tensor<T> &rhs);
    Tensor<T> &operator-=(const Tensor<T> &rhs);

    void clear();
    bool all_zero() const;
};

}  // namespace topart


#endif  // TENSOR_H_
