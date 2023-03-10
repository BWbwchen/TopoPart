/* Copyright (C) 2023 Bo-Wei Chen<time.chenbw@gmail.com> - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#ifndef DB_H
#define DB_H

#include <fstream>
#include <queue>

#include "graph.h"
#include "node.h"
#include "timer.h"

namespace topart {

using std::fstream;
using std::queue;

class DB {
public:
    Graph<FPGANode> fpga;
    Graph<CircuitNode> circuit;
    vector<Net *> nets;
    vector<intg> fpga_neighbor_free_space;

    Timer timer;

    // topart procedure - algorithm 1
    void cal_circuit_candidate(
        queue<pair<intg, intg>> Q,
        unordered_map<intg, unordered_map<intg, intg>> circuit_node_s_dist);

    intg estimate_cut_increment(intg node_id, intg to_fpga_id);
    bool enough_space_for_neighbor(CircuitNode *c, FPGANode *f);

    void calculate_fpga_neighbor_free_space(intg fpga_id);
    void output_loss();
    intg topo_vio(FPGANode *f, intg c);

    void try_legalize(intg threshold);

public:
    DB() { timer.set_start(); }

    // Build DB
    void build_fpga_graph(intg num_vertex,
                          intg capacity,
                          vector<pair<intg, intg>> &e);
    void build_circuit_graph(intg num_vertex, vector<vector<intg>> &e);
    void set_fixed_circuit(vector<pair<intg, intg>> &e);

    // topart procedure
    void calculate_candidate_fpga();
    void partition();
    void refine();
    void output(fstream &out);
};


}  // namespace topart

#endif /* DB_H */
