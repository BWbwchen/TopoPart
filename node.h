#ifndef NODE_H_
#define NODE_H_

#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

#include "type.h"

namespace topart {

using std::endl;
using std::set;
using std::string;
using std::stringstream;
using std::unordered_map;

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
    unordered_map<intg, set<FPGANode *>> S_hat;

public:
    FPGANode(intg s, intg c) : Node(s), capacity(c) {}
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
};

class CircuitNode : public Node {
public:
    FPGANode *fpga_node;

    set<FPGANode *> cddt;  // candidate fpga for this node to be assigned to.

    bool fixed;
    set<CircuitNode *> S;  // only for fixed node.

public:
    CircuitNode(intg s) : Node(s) {
        fixed = false;
        fpga_node = nullptr;
    }
    void set_fixed(FPGANode *fn) {
        fixed = true;
        fpga_node = fn;
    }

    bool is_fixed() { return fixed; }
    virtual string status() override {
        // Do nothing
        return "CircuitNode didn't need this right now.";
    }
};



}  // namespace topart


#endif  // NODE_H_
