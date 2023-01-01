#include "graph.h"

namespace topart {

template <class T>
void Graph<T>::init(vector<T *> &vv, vector<vector<T *>> &gg) {
    num_vertex = gg.size();
    v = vv;
    g = gg;
}

template class Graph<FPGANode>;
template class Graph<CircuitNode>;

}  // namespace topart
