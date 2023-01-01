#ifndef GRAPH_H_
#define GRAPH_H_

#include <unordered_map>
#include <utility>
#include <vector>

#include "node.h"

namespace topart {

using std::pair;
using std::unordered_map;
using std::vector;

template <class T>
class Graph {
public:
    intg num_vertex;
    vector<T *> v;
    vector<vector<T *>> g;
    vector<intg> max_dist;

public:
    Graph() {}
    void init(vector<T *> &vv, vector<vector<T *>> &gg);
    vector<T *> get_all_vertex() { return v; }
    void dijkstra();
};


}  // namespace topart

#endif  // GRAPH_H_
