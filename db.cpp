#include "db.h"

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>

#include "log.h"
#include "node.h"

namespace topart {

using std::cout;
using std::endl;
using std::fstream;
using std::make_pair;
using std::queue;


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
        auto &ff = p.second;
        circuit_vertex[fixed_circuit]->set_fixed(fpga_vertex[ff]);
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
        fpga_list[src]->S_hat[dist].emplace(fpga_list[src]);
    };
    fpga.calculate_max_dist(insert_S_hat);
    fpga.get_status();
    log("FINISH fpga pre-calculation");

    queue<pair<intg, intg>> q;
    // vi -> (vj , dist(vi, vj))
    unordered_map<intg, unordered_map<intg, intg>> circuit_node_s_dist;
    // calculate circuit node's S set
    const auto &circuit_list = circuit.get_all_vertex();
    for (auto &c_node : circuit.get_all_vertex()) {
        if (c_node->is_fixed()) {
            q.push(make_pair(c_node->name, c_node->fpga_node->name));
            auto &d = fpga.max_dist[c_node->fpga_node->name];
            circuit.get_max_dist(c_node->name,
                                 [&](intg src, intg dst, intg dist) {
                                     circuit_node_s_dist[src][dst] = dist;
                                     if (dist < d) {
                                         c_node->S.emplace(circuit_list[dst]);
                                     }
                                     c_node->S.emplace(circuit_list[src]);
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

    cal_circuit_candidate(std::move(q), std::move(circuit_node_s_dist));
}

void DB::cal_circuit_candidate(
    queue<pair<intg, intg>> Q,
    unordered_map<intg, unordered_map<intg, intg>> circuit_node_s_dist) {
    vector<FPGANode *> intersection_result;
    while (!Q.empty()) {
        auto &top = Q.front();
        Q.pop();

        const auto &node_id = top.first;
        const auto &fpga_id = top.second;
        const auto &c_node = circuit.get_vertex(node_id);
        const auto &fpga_node = fpga.get_vertex(fpga_id);

        auto d = fpga.max_dist[fpga_id];

        for (auto &neighbor : c_node->S) {
            if (neighbor->is_fixed())
                continue;

            intg k = -1;
            if (circuit_node_s_dist.count(node_id) > 0 &&
                circuit_node_s_dist[node_id].count(neighbor->name) > 0) {
                k = circuit_node_s_dist[node_id][neighbor->name];
            } else {
                throw std::logic_error("Didn't calculate the dist");
            }

            intg desired_size =
                std::min(neighbor->cddt.size(), fpga_node->S_hat[k].size());
            if (intersection_result.size() < desired_size)
                intersection_result.resize(desired_size);

            auto it = std::set_intersection(
                neighbor->cddt.begin(), neighbor->cddt.end(),
                fpga_node->S_hat[k].begin(), fpga_node->S_hat[k].end(),
                intersection_result.begin());

            neighbor->cddt = set<FPGANode *>(intersection_result.begin(), it);

            if (neighbor->cddt.size() == 1) {
                auto vj_hat = *(neighbor->cddt.begin());
                Q.push(make_pair(neighbor->name, vj_hat->name));
                // NOTE: remember to calculate the dist with other
                circuit.get_max_dist(neighbor->name,
                                     [&](intg src, intg dst, intg dist) {
                                         circuit_node_s_dist[src][dst] = dist;
                                         // if (dist < d) {
                                         //     neighbor->S.emplace(circuit.get_vertex(dst));
                                         // }
                                         // neighbor->S.emplace(circuit.get_vertex(src));
                                     });
            }

            if (neighbor->cddt.size() == 0) {
                throw std::logic_error("Unreachable");
            }
        }
    }

#ifdef LOG
    log("Cddt result: ");
    for (auto &c : circuit.get_all_vertex()) {
        cout << "Cddt(" << c->name << ") = ";
        for (auto &s : c->cddt) {
            cout << s->name << ", ";
        }
        cout << endl;
    }
#endif
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
