#ifndef GRAPH_H_
#define GRAPH_H_

#include <limits>
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
    unordered_map<T *, intg> v_map;
    vector<vector<T *>> g;
    vector<intg> max_dist;

    intg get_max_dist(intg start);

public:
    Graph() {}
    void init(vector<T *> &vv, vector<vector<T *>> &gg);
    vector<T *> get_all_vertex() { return v; }
    void calculate_max_dist();

    void get_status();
};


}  // namespace topart

#endif  // GRAPH_H_
