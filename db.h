#ifndef DB_H
#define DB_H

#include <fstream>
#include <queue>

#include "graph.h"
#include "node.h"

namespace topart {

using std::fstream;
using std::queue;

class DB {
public:
    Graph<FPGANode> fpga;
    Graph<CircuitNode> circuit;

    // topart procedure - algorithm 1
    void cal_circuit_candidate(
        queue<pair<intg, intg>> Q,
        unordered_map<intg, unordered_map<intg, intg>> circuit_node_s_dist);

public:
    DB() {}

    // Build DB
    void build_fpga_graph(intg num_vertex,
                          intg capacity,
                          vector<pair<intg, intg>> &e);
    void build_circuit_graph(intg num_vertex, vector<vector<intg>> &e);
    void set_fixed_circuit(vector<pair<intg, intg>> &e);

    // topart procedure
    void calculate_candidate_fpga();
    void output(fstream &out);
};


}  // namespace topart

#endif /* DB_H */
