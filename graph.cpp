#include "graph.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <utility>

namespace topart {

using std::cout;
using std::endl;
using std::make_pair;
using std::max;
using std::pair;
using std::queue;

template <class T>
void Graph<T>::init(vector<T *> &vv, vector<vector<T *>> &gg) {
    num_vertex = gg.size();
    v = vv;
    for (int i = 0; i < vv.size(); ++i) {
        v_map[vv[i]] = i;
    }
    g = gg;
}

template <class T>
void Graph<T>::calculate_max_dist() {
    max_dist = vector<intg>(num_vertex, 0);
    for (int i = 0; i < num_vertex; ++i) {
        max_dist[i] = get_max_dist(i);
    }
}

template <class T>
intg Graph<T>::get_max_dist(intg start) {
    // bfs for max dist
    vector<bool> done;
    done.resize(num_vertex, false);

    intg ans = std::numeric_limits<intg>::min();
    // node id, depth
    queue<pair<intg, intg>> q;
    q.push(make_pair(start, 0));

    done[start] = true;
    while (!q.empty()) {
        auto top = q.front();
        auto &id = top.first;
        auto &depth = top.second;
        q.pop();

        ans = max(ans, depth);

        for (auto &neighbor : g[id]) {
            auto neighbor_id = v_map[neighbor];
            if (done[neighbor_id] == false) {
                q.push(make_pair(neighbor_id, depth + 1));
                done[neighbor_id] = true;
            }
        }
    }
    return ans;
}

template <class T>
void Graph<T>::get_status() {
    for (int i = 0; i < num_vertex; ++i) {
        cout << i << "'s max dist is: " << max_dist[i] << endl;
    }
}


template class Graph<FPGANode>;
template class Graph<CircuitNode>;

}  // namespace topart
