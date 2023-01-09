/* Copyright (C) 2023 Bo-Wei Chen<time.chenbw@gmail.com> - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include "tensor.h"

#include <cassert>

namespace topart {

template <class T>
T &Tensor<T>::at(intg i) {
    assert(i < v.size());
    return v[i];
}

template <class T>
Tensor<T> &Tensor<T>::operator+=(const Tensor<T> &rhs) {
    assert(v.size() == rhs.v.size());
    for (intg i = 0; i < v.size(); ++i) {
        v[i] += rhs.v[i];
    }
    return *this;
}

template <class T>
Tensor<T> &Tensor<T>::operator-=(const Tensor<T> &rhs) {
    assert(v.size() == rhs.v.size());
    for (intg i = 0; i < v.size(); ++i) {
        v[i] -= rhs.v[i];
    }
    return *this;
}


template <class T>
void Tensor<T>::clear() {
    intg size = v.size();
    v.clear();
    v.resize(size, 0);
}

template <class T>
bool Tensor<T>::all_zero() const {
    bool ret = true;
    for (const auto &vv : v) {
        ret &= (vv <= 0);
    }
    return ret;
}


template class Tensor<intg>;

}  // namespace topart
