#ifndef GRAPH_H_
#define GRAPH_H_

#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "node.h"

namespace topart {

using std::pair;
using std::unordered_map;
using std::unordered_set;
using std::vector;


// src, dst, depth -> !
using Func = std::function<void(intg, intg, intg)>;

template <class T>
class Graph {
public:
    intg num_vertex;
    vector<T *> v;
    unordered_map<T *, intg> v_map;
    vector<vector<T *>> g;
    vector<unordered_set<T *>> g_set;
    vector<intg> max_dist;


public:
    Graph() {}
    void init(vector<T *> &vv, vector<vector<T *>> &gg);
    vector<T *> get_all_vertex() { return v; }
    void calculate_max_dist(Func f);
    intg get_max_dist(intg start, Func f);
    T *get_vertex(intg i);

    void get_status();
};


}  // namespace topart

#endif  // GRAPH_H_
