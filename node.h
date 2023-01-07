#ifndef NODE_H_
#define NODE_H_

#include <algorithm>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "tensor.h"
#include "type.h"

namespace topart {

using std::endl;
using std::set;
using std::string;
using std::stringstream;
using std::unordered_map;
using std::unordered_set;

class Net;

class Node {
public:
    intg name;

public:
    Node(intg s) : name(s){};
    virtual string status() = 0;
};

class FPGANode : public Node {
public:
    intg capacity;
    intg usage;
    unordered_map<intg, set<FPGANode *>> S_hat;

public:
    FPGANode(intg s, intg c) : Node(s), capacity(c), usage(0) {}
    virtual string status() override {
        stringstream ss;
        for (auto &p : S_hat) {
            auto &dist = p.first;
            auto &ff = p.second;
            ss << "hat{S}(F" << name << ", " << dist << ") = ";
            for (auto &s : ff) {
                ss << s->name << ", ";
            }
            ss << endl;
        }
        return ss.str();
    }
    void add_circuit() { usage++; }
    void remove_circuit() { usage--; }
    bool valid() { return usage < capacity; }
    intg free_space() { return capacity - usage; }
};

class CircuitNode : public Node {
public:
    FPGANode *fpga_node;

    set<FPGANode *> cddt;  // candidate fpga for this node to be assigned to.
    Tensor<intg> tsr_cddt;

    bool fixed;
    set<CircuitNode *> S;  // only for fixed node.

    unordered_set<Net *> nets;
    unordered_map<intg, intg> cut_increment_map;

    bool should_defer;


public:
    CircuitNode(intg s, intg num_f) : Node(s) {
        fixed = false;
        fpga_node = nullptr;
        tsr_cddt = Tensor<intg>(num_f);
        should_defer = false;
        cut_increment_map.clear();
    }
    void set_fixed(FPGANode *fn);
    void add_net(Net *n) { nets.emplace(n); }
    void add_fpga(FPGANode *fn);
    void remove_fpga();

    void reset_cut_increment() { cut_increment_map.clear(); }
    void calculate_cut_increment(FPGANode *if_f,
                                 unordered_set<CircuitNode *> neighbor_lists);

    bool is_fixed() { return fixed; }
    virtual string status() override {
        stringstream ss;
        ss << "S(" << name << ") = ";
        for (auto &p : S) {
            ss << p->name << ", ";
        }
        ss << endl;
        return ss.str();
    }

    void flush_tsr_to_cddt(vector<FPGANode *> &mapping);
    void flush_cddt_to_tsr();

    bool assigned() { return is_fixed() || fpga_node != nullptr; }
    void defer() { should_defer = true; }

    intg try_move();
    intg try_move(FPGANode *f);
};


class Net {
public:
    unordered_set<CircuitNode *> net_cell;
    unordered_set<FPGANode *> used_fpga_node;
    unordered_map<intg, intg> used_fpga_node_count;

public:
    Net(unordered_set<CircuitNode *> nc) : net_cell(nc){};
    void add_fpga(FPGANode *f) {
        used_fpga_node.emplace(f);
        if (used_fpga_node_count.count(f->name) <= 0) {
            used_fpga_node_count[f->name] = 1;
        } else {
            used_fpga_node_count[f->name]++;
        }
    }
    void remove_fpga(FPGANode *f) {
        if (used_fpga_node_count[f->name] == 1) {
            used_fpga_node_count.erase(f->name);
            used_fpga_node.erase(f);
        } else {
            used_fpga_node_count[f->name] -= 1;
        }
    }
    intg estimate_increase_cut_size(FPGANode *if_f);
    intg estimate_increase_cut_size_refine(FPGANode *if_f);
    intg cost() {
        if (used_fpga_node.size() <= 1)
            return 0;
        return used_fpga_node.size();
    }
    bool try_move(FPGANode *f);
};



}  // namespace topart


#endif  // NODE_H_
