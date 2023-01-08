#include "db.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
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
    fpga_neighbor_free_space.resize(num_vertex, 0);
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

    // Build Net and graph
    nets.clear();
    unordered_set<CircuitNode *> us;

    for (auto &n : e) {
        intg start_v = n[0];

        unordered_set<CircuitNode *> us;
        us.clear();
        us.emplace(circuit_node[start_v]);

        for (int i = 1; i < n.size(); ++i) {
            auto &neighbor = n[i];
            us.emplace(circuit_node[neighbor]);
            gg[start_v].emplace_back(circuit_node[neighbor]);
            gg[neighbor].emplace_back(circuit_node[start_v]);
        }
        nets.push_back(new Net(us));
        circuit_node[start_v]->add_net(nets.back());
        for (int i = 1; i < n.size(); ++i) {
            auto &neighbor = n[i];
            circuit_node[neighbor]->add_net(nets.back());
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

void DB::calculate_fpga_neighbor_free_space(intg fpga_id) {
    auto &f_usage = fpga_neighbor_free_space[fpga_id];
    const auto &f = fpga.get_vertex(fpga_id);

    f_usage = f->free_space();
    for (auto &f_neighbor : fpga.g[f->name]) {
        f_usage += f_neighbor->free_space();
    }
}

void DB::partition() {
    // NOTE: for algorithm 2, Q with size of cddt of node id, node id
    // It need to be sorted.
    auto Qcmp = [&](const intg &lhs, const intg &rhs) {
        if (lhs == rhs)
            return false;

        intg lg = circuit.g_set[lhs].size();
        intg rg = circuit.g_set[rhs].size();

        intg lcs = circuit.get_vertex(lhs)->cddt.size() + lg;
        intg rcs = circuit.get_vertex(rhs)->cddt.size() + rg;

        if (lcs == rcs) {
            if (lg == rg) {
                return lhs < rhs;
            }
            return lg < rg;
        }

        return lcs < rcs;
    };
    using QType = set<intg, decltype(Qcmp)>;

    // NOTE: for algorithm 2, R with element pair:
    // node_id, fpga_id
    intg hp = 0;
    intg capacity_digit = log10(fpga.get_vertex(0)->capacity) + 1;
    intg threshold = 2;
    if (capacity_digit <= threshold) {
        hp = 1;
    } else {
        hp = 2 * (intg) pow(10, capacity_digit - threshold);
    }
    auto Rcmp = [&](const pair<intg, intg> &lhs, const pair<intg, intg> &rhs) {
        const auto &cc = circuit.get_vertex(lhs.first);
        const auto &fl = fpga.get_vertex(lhs.second);
        const auto &fr = fpga.get_vertex(rhs.second);
        assert(cc->cut_increment_map.count(lhs.second));
        assert(cc->cut_increment_map.count(rhs.second));
        auto l_cost = cc->cut_increment_map[lhs.second];
        auto r_cost = cc->cut_increment_map[rhs.second];

        intg fl_usage = fpga_neighbor_free_space[lhs.second];
        intg fr_usage = fpga_neighbor_free_space[rhs.second];

        auto ll = (l_cost + fl->usage / hp);
        auto rr = (r_cost + fr->usage / hp);
        if (ll == rr) {
            return fl_usage > fr_usage;
        }
        return ll < rr;
    };
    using RType = set<pair<intg, intg>, decltype(Rcmp)>;


    QType Q(Qcmp);
    vector<RType> R(circuit.num_vertex, RType(Rcmp));
    for (auto &c : circuit.get_all_vertex()) {
        if (c->is_fixed())
            continue;

        c->fpga_node = nullptr;
        Q.emplace(c->name);
    }

    Tensor<intg> v_cddt(fpga.num_vertex);
    while (!Q.empty()) {
        auto node_vj = *(Q.begin());
        Q.erase(Q.begin());
        auto c = circuit.get_vertex(node_vj);
        if (c->assigned())
            continue;

        stringstream ss;
        ss << "Deal with circuit node: " << node_vj << "(" << c->cddt.size()
           << ")\t place it to ";

        if (c->cddt.size() == 0) {
            c->defer();
            continue;
        }

        R[c->name].clear();
        c->reset_cut_increment();
        for (auto &ff : c->cddt) {
            calculate_fpga_neighbor_free_space(ff->name);
            c->calculate_cut_increment(ff, circuit.g_set[c->name]);
            R[c->name].emplace(make_pair(c->name, ff->name));
        }

        assert(node_vj == c->name);
        bool traceback = false;

        FPGANode *f = nullptr;
        intg fpga_vj;
        do {
            if (R[node_vj].size() <= 0) {
                c->defer();
                goto end;
            }

            fpga_vj = (*(R[node_vj].begin())).second;
            f = fpga.get_vertex(fpga_vj);
            R[node_vj].erase(R[node_vj].begin());
        } while (f->valid() == false);

        assert(f->valid());

        ss << fpga_vj << "(" << f->usage << "/" << f->capacity << ")"
           << "\t";

        ss << " and Q size: " << Q.size()
           << " and R size(poped): " << R[node_vj].size() << endl;


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
            c->cddt.erase(f);
            c->flush_cddt_to_tsr();
            c->fpga_node = nullptr;
            Q.emplace(c->name);
            for (auto &neighbor : circuit.g[c->name]) {
                if (neighbor->assigned())
                    continue;

                neighbor->tsr_cddt += v_cddt;
            }
            R[c->name].clear();
        } else {
            c->add_fpga(f);
            f->add_circuit();
            for (auto &neighbor : circuit.g[c->name]) {
                if (neighbor->assigned() || neighbor->should_defer)
                    continue;

                // push into Q
                assert(*(Q.find(neighbor->name)) == neighbor->name);
                Q.erase(neighbor->name);

                neighbor->flush_tsr_to_cddt(fpga.v);

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
                    c->add_fpga(tmp_f);
                    tmp_f->add_circuit();
                    break;
                }
                fpga_id++;
            }
        } else {
            intg cut_increment = std::numeric_limits<intg>::max();
            FPGANode *best_f = nullptr;
            for (auto &tmp_f : c->cddt) {
                c->calculate_cut_increment(tmp_f, circuit.g_set[c->name]);
                if (cut_increment > c->cut_increment_map[tmp_f->name]) {
                    cut_increment = c->cut_increment_map[tmp_f->name];
                    best_f = tmp_f;
                }
            }
            assert(best_f != nullptr);
            c->add_fpga(best_f);
            best_f->add_circuit();
            // c->add_fpga(*(c->cddt.begin()));
            // (*(c->cddt.begin()))->add_circuit();
        }
        // assert(c->cddt.size() > 0);
    }

    if (need_refine)
        cout << "[BW] Refine " << need_refine << "/" << circuit.num_vertex
             << " circuit nodes. with force assigned: " << force_assign << endl;


    output_loss();
    // Refine with move-based. We try to move the boundary node, which is the
    // node that in the FPGA node that different with other node in the same
    // net.
    intg total_dec = 0;
    intg total_topo_vio = 0;
    for (int i = 0; i < 2; ++i) {
        intg tmp_dec = std::numeric_limits<intg>::max();
        FPGANode *refine_f = nullptr;

        vector<CircuitNode *> re_q;
        for (auto &c : circuit.get_all_vertex()) {
            if (c->is_fixed() || c->try_move() == false)
                continue;
            re_q.emplace_back(c);
        }
        sort(re_q.begin(), re_q.end(),
             [&](const CircuitNode *lhs, const CircuitNode *rhs) {
                 return lhs->nets.size() > rhs->nets.size();
             });

        for (auto &c : re_q) {
            if (c->is_fixed() || c->try_move() == false)
                continue;

            tmp_dec = std::numeric_limits<intg>::max();
            refine_f = nullptr;
            for (auto &neighbor : circuit.g_set[c->name]) {
                auto &nf = neighbor->fpga_node;
                if (nf == c->fpga_node || nf->valid() == false)
                    continue;

                intg td = c->try_move(nf);
                if (tmp_dec > td) {
                    tmp_dec = td;
                    refine_f = nf;
                }
            }
            if (refine_f == nullptr)
                continue;

            intg topology_violation_increase = 0;
            for (auto &neighbor : circuit.g_set[c->name]) {
                if (fpga.g_set[refine_f->name].count(
                        fpga.get_vertex(neighbor->fpga_node->name)) <= 0 &&
                    refine_f != neighbor->fpga_node)
                    topology_violation_increase += 2;
            }

            if (tmp_dec + topology_violation_increase < 0) {
                c->fpga_node->remove_circuit();
                c->remove_fpga();
                c->add_fpga(refine_f);
                refine_f->add_circuit();
                total_dec += tmp_dec;
                total_topo_vio += topology_violation_increase;
            }
        }
    }

    cout << "[BW] dec: " << total_dec << endl;
    cout << "[BW] topo vio: " << total_topo_vio << endl;
    output_loss();
}

void DB::output_loss() {
    const auto &circuit_vertex = circuit.get_all_vertex();
    intg topology_cost = 0;
    for (auto &n : nets) {
        topology_cost += n->cost();
    }

    intg penalty = 0;
    for (auto &c : circuit_vertex) {
        for (auto &neighbor : circuit.g_set[c->name]) {
            if (fpga.g_set[c->fpga_node->name].count(
                    fpga.get_vertex(neighbor->fpga_node->name)) <= 0 &&
                c->fpga_node != neighbor->fpga_node)
                penalty += 1;
        }
    }

    cout << "=========================================================" << endl;
    cout << "[BW] \tTopology Cost is: " << topology_cost << endl;
    cout << "\tPenalty Cost is: " << penalty << endl;
    cout << "\tTOTAL Cost is: " << topology_cost + penalty << endl;
    cout << "=========================================================" << endl;
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

#ifdef LOG
    output_loss();
#endif

    if (not_set == 0)
        return;
    cout << "=========================================================" << endl;
    cout << "[BW] Total Not Set: " << not_set << "/" << circuit.num_vertex
         << endl;
    cout << "=========================================================" << endl;
}

}  // namespace topart
