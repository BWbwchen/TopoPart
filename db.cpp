#include "db.h"

#include <string>

#include "log.h"
#include "node.h"

namespace topart {

using std::fstream;

void DB::build_fpga_graph(intg num_vertex,
                          intg capacity,
                          vector<pair<intg, intg>> &e) {
    vector<vector<FPGANode *>> gg(num_vertex);
    vector<FPGANode *> fpga_node;
    for (int i = 0; i < num_vertex; ++i) {
        fpga_node.emplace_back(new FPGANode(i, capacity));
    }

    for (auto &p : e) {
        auto &v = p.first;
        auto &w = p.second;
        gg[v].emplace_back(fpga_node[w]);
        gg[w].emplace_back(fpga_node[v]);
    }
    fpga.init(fpga_node, gg);
}

void DB::build_circuit_graph(intg num_vertex, vector<vector<intg>> &e) {
    vector<vector<CircuitNode *>> gg(num_vertex);
    vector<CircuitNode *> circuit_node;
    for (int i = 0; i < num_vertex; ++i) {
        circuit_node.emplace_back(new CircuitNode(i));
    }

    for (int i = 0; i < num_vertex; ++i) {
        auto &neighbor_list = e[i];
        for (auto &neighbor : neighbor_list) {
            gg[i].emplace_back(circuit_node[neighbor]);
        }
    }

    circuit.init(circuit_node, gg);
}

void DB::set_fixed_circuit(vector<pair<intg, intg>> &e) {
    const auto &circuit_vertex = circuit.get_all_vertex();
    const auto &fpga_vertex = fpga.get_all_vertex();
    for (auto &p : e) {
        auto &fixed_circuit = p.first;
        auto &fpga = p.second;
        circuit_vertex[fixed_circuit]->set_fixed(fpga_vertex[fpga]);
    }
}

void DB::calculate_candidate_fpga() {
    // calculate fpga maxDist and S-hat
    const auto &fpga_list = fpga.get_all_vertex();
    auto insert_S_hat = [&](intg src, intg dst, intg dist) {
        for (int i = 1; i < dist; ++i) {
            for (auto &s : fpga_list[src]->S_hat[i]) {
                fpga_list[src]->S_hat[dist].emplace(s);
            }
        }
        fpga_list[src]->S_hat[dist].emplace(fpga_list[dst]);
    };
    fpga.calculate_max_dist(insert_S_hat);
    fpga.get_status();
    log("FINISH fpga pre-calculation");

    // calculate circuit node's S set
    const auto &circuit_list = circuit.get_all_vertex();
    for (auto &c_node : circuit.get_all_vertex()) {
        if (c_node->is_fixed()) {
            auto &d = fpga.max_dist[c_node->fpga_node->name];
            circuit.get_max_dist(c_node->name,
                                 [&](intg src, intg dst, intg dist) {
                                     if (dist < d) {
                                         c_node->S.emplace(circuit_list[dst]);
                                     }
                                 });
        }
    }
    log("FINISH circuit fixed node pre-calculation");

    for (auto &c_node : circuit.get_all_vertex()) {
        if (c_node->is_fixed()) {
            c_node->cddt.emplace(c_node->fpga_node);
        } else {
            for (auto &f : fpga_list) {
                c_node->cddt.emplace(f);
            }
        }
    }
    log("FINISH circuit node pre-calculation");

    circuit.get_status();

    // TODO:
}


void DB::output(fstream &out) {
    const auto &circuit_vertex = circuit.get_all_vertex();
    for (auto &c : circuit_vertex) {
        if (c->is_fixed()) {
            out << c->name << " " << c->fpga_node->name << std::endl;
        } else {
            out << c->name << " "
                << "0" << std::endl;
        }
    }
}
}  // namespace topart
