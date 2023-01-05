#include "db.h"

#include <algorithm>
#include <cassert>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>

#include "log.h"
#include "node.h"
#include "tensor.h"

namespace topart {

using std::cout;
using std::endl;
using std::fstream;
using std::make_pair;
using std::pair;
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
        circuit_node.emplace_back(new CircuitNode(i, fpga.num_vertex));
    }

    for (int i = 0; i < num_vertex; ++i) {
        auto &neighbor_list = e[i];
        for (auto &neighbor : neighbor_list) {
            gg[i].emplace_back(circuit_node[neighbor]);
            gg[neighbor].emplace_back(circuit_node[i]);
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
        fpga_vertex[ff]->add_circuit();
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

    for (auto &c : circuit.get_all_vertex()) {
        c->flush_cddt_to_tsr();
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

intg DB::estimate_cut_increment(intg node_id, intg to_fpga_id) {
    // TODO: calculate cut size increment
    // check if fixed node in f is the neighbor of c
    //     Yes, no cut size increment
    //     No, +1
    intg cut_size_increment = 0;
    const auto &to_fpga = fpga.get_vertex(to_fpga_id);
    for (auto &neighbor : circuit.g[node_id]) {
        if (neighbor->assigned() == false || neighbor->fpga_node != to_fpga)
            cut_size_increment++;
    }

    const auto &c = circuit.get_vertex(node_id);
    const auto &f = fpga.get_vertex(to_fpga_id);
    intg demand = 1;
    for (auto &c_neighbor : circuit.g[c->name]) {
        if (c_neighbor->assigned())
            continue;
        demand++;
    }

    intg supply = f->free_space();
    for (auto &f_neighbor : fpga.g[f->name]) {
        supply += f_neighbor->free_space();
    }
    // return supply - demand;

    return supply;
    // return std::max(cut_size_increment, (intg)
    // fpga.g_set[to_fpga_id].size()); return cut_size_increment; return
    // fpga.g_set[to_fpga_id].size(); return demand;
}

bool DB::enough_space_for_neighbor(CircuitNode *c, FPGANode *f) {
    intg demand = 1;
    for (auto &c_neighbor : circuit.g[c->name]) {
        if (c_neighbor->assigned())
            continue;
        demand++;
    }

    intg supply = f->free_space();
    for (auto &f_neighbor : fpga.g[f->name]) {
        supply += f_neighbor->free_space();
    }
    return demand <= supply;
}

void DB::partition() {
    // NOTE: for algorithm 2, Q with size of cddt of node id, node id
    // It need to be sorted.
    auto Qcmp = [&](const intg &lhs, const intg &rhs) {
        if (lhs == rhs)
            return false;

        intg lcs = circuit.get_vertex(lhs)->cddt.size();
        intg rcs = circuit.get_vertex(rhs)->cddt.size();
        if (lcs == rcs) {
            return lhs < rhs;
        }

        return lcs < rcs;
    };
    using QType = set<intg, decltype(Qcmp)>;

    // NOTE: for algorithm 2, R with increment of cut size if circuit was in
    // fpga id, fpga id It need to be sorted.
    using RType = set<pair<intg, intg>, std::greater<pair<intg, intg>>>;


    QType Q(Qcmp);
    vector<RType> R(circuit.num_vertex);
    for (auto &c : circuit.get_all_vertex()) {
        if (c->is_fixed())
            continue;

        c->fpga_node = nullptr;
        Q.emplace(c->name);

        for (auto &f : c->cddt) {
            intg cut_size_increment = estimate_cut_increment(c->name, f->name);
            R[c->name].emplace(make_pair(cut_size_increment, f->name));
        }
    }

    Tensor<intg> v_cddt(fpga.num_vertex);
    while (!Q.empty()) {
        auto node_vj = *(Q.begin());
        Q.erase(Q.begin());
        if (circuit.get_vertex(node_vj)->assigned())
            continue;

        stringstream ss;
        ss << "Deal with circuit node: " << node_vj << "("
           << circuit.get_vertex(node_vj)->cddt.size() << ")\t place it to ";

        if (R[node_vj].size() <= 0) {
            circuit.get_vertex(node_vj)->defer();
            continue;
        }
        assert(R[node_vj].size() > 0);
        auto r_top = *(R[node_vj].begin());
        auto fpga_vj = r_top.second;
        R[node_vj].erase(R[node_vj].begin());

        auto c = circuit.get_vertex(node_vj);
        auto f = fpga.get_vertex(fpga_vj);

        bool traceback = false;

        // TODO: not only valid, also enough space for circuit node's neighbor
        // while (f->valid() == false ||
        //        enough_space_for_neighbor(c, f) == false) {
        while (f->valid() == false) {
            if (R[node_vj].size() <= 0) {
                c->defer();
                goto end;
            }
            assert(R[node_vj].size() > 0);
            r_top = *(R[node_vj].begin());
            fpga_vj = r_top.second;
            f = fpga.get_vertex(fpga_vj);
            R[node_vj].erase(R[node_vj].begin());
        }
        assert(f->valid());

        ss << fpga_vj << "(" << f->usage << "/" << f->capacity << ")"
           << "\t";

        ss << " and Q size: " << Q.size()
           << " and R size(poped): " << R[node_vj].size() << endl;


        c->fpga_node = f;
        f->add_circuit();
        v_cddt.clear();
        // the fpga node, which didn't directly connect to f
        for (auto &indirect_f : fpga.get_all_vertex()) {
            if (fpga.g_set[f->name].count(indirect_f) <= 0)
                v_cddt.at(indirect_f->name) += 1;
        }
        v_cddt.at(fpga_vj) = 0;


        for (auto &neighbor : circuit.g[c->name]) {
            if (neighbor->assigned() || neighbor->should_defer)
                continue;

            neighbor->tsr_cddt -= v_cddt;

            if (neighbor->tsr_cddt.all_zero()) {
                traceback = true;
            }
        }

        if (traceback) {
            c->cddt.erase(c->fpga_node);
            c->flush_cddt_to_tsr();
            c->fpga_node = nullptr;
            f->remove_circuit();
            Q.emplace(c->name);
            for (auto &neighbor : circuit.g[c->name]) {
                if (neighbor->assigned())
                    continue;

                neighbor->tsr_cddt += v_cddt;
            }
            R[c->name].clear();
            for (auto &cddt_fpga : c->cddt) {
                intg cut_size_increment =
                    estimate_cut_increment(c->name, cddt_fpga->name);
                R[c->name].emplace(
                    make_pair(cut_size_increment, cddt_fpga->name));
            }
        } else {
            for (auto &neighbor : circuit.g[c->name]) {
                if (neighbor->assigned() || neighbor->should_defer)
                    continue;

                ss << "\t\t";
                ss << neighbor->name << "'s cddt: ";

                // push into Q
                assert(*(Q.find(neighbor->name)) == neighbor->name);
                Q.erase(neighbor->name);

                neighbor->flush_tsr_to_cddt(fpga.v);

                R[neighbor->name].clear();
                for (auto &cddt_fpga : neighbor->cddt) {
                    ss << cddt_fpga->name << ", ";
                    intg cut_size_increment =
                        estimate_cut_increment(neighbor->name, cddt_fpga->name);
                    R[neighbor->name].emplace(
                        make_pair(cut_size_increment, cddt_fpga->name));
                }
                ss << endl;

                Q.emplace(neighbor->name);
            }
        }
    end:
        log(ss.str());

        if (traceback)
            log("Traceback");
        else
            log("Don't traceback");
    }
}

void DB::refine() {
    cout << "Start refine." << endl;
    // make the un-assigned node assigned

    intg need_refine = 0;
    intg force_assign = 0;
    intg fpga_id = 0;
    for (auto &c : circuit.get_all_vertex()) {
        if (c->assigned())
            continue;

        need_refine++;
        c->fpga_node = nullptr;
        c->should_defer = false;

        c->cddt.clear();
        for (auto &neighbor : circuit.g[c->name]) {
            if (neighbor->assigned() && neighbor->fpga_node->valid())
                c->cddt.emplace(neighbor->fpga_node);
        }

        if (c->cddt.size() == 0) {
            // greedy assign.
            // TODO: can I place the nearest fpga node?
            force_assign++;
            while (fpga_id < fpga.num_vertex) {
                const auto &tmp_f = fpga.get_vertex(fpga_id);
                if (tmp_f->valid()) {
                    tmp_f->add_circuit();
                    c->fpga_node = tmp_f;
                    break;
                }
                fpga_id++;
            }
        } else {
            for (auto &tmp_f : c->cddt) {
                if (tmp_f->valid()) {
                    tmp_f->add_circuit();
                    c->fpga_node = tmp_f;
                    break;
                }
            }
        }
        // assert(c->cddt.size() > 0);
    }

    if (need_refine)
        cout << "[BW] Refine " << need_refine << "/" << circuit.num_vertex
             << " circuit nodes. with force assigned: " << force_assign << endl;
}


void DB::output(fstream &out) {
    const auto &circuit_vertex = circuit.get_all_vertex();
    intg not_set = 0;
    for (auto &c : circuit_vertex) {
        // assert(c->fpga_node != nullptr);
        if (c->fpga_node == nullptr) {
            not_set++;
            continue;
        }
        out << c->name << " " << c->fpga_node->name << std::endl;
    }
    if (not_set == 0)
        return;
    cout << "=========================================================" << endl;
    cout << "[BW] Total Not Set: " << not_set << "/" << circuit.num_vertex
         << endl;
    cout << "=========================================================" << endl;
}

}  // namespace topart
