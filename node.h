#ifndef NODE_H_
#define NODE_H_

#include <set>
#include <string>
#include <unordered_map>

#include "type.h"

namespace topart {

using std::set;
using std::string;
using std::unordered_map;

class Node {
public:
    string name;

public:
    Node(string s) : name(s){};
};

class FPGANode : public Node {
public:
    intg capacity;
    unordered_map<intg, set<FPGANode *>> S_hat;

public:
    FPGANode(string s, intg c) : Node(s), capacity(c) {}
};

class CircuitNode : public Node {
public:
    FPGANode *fpga_node;

    set<FPGANode *> cddt;  // candidate fpga for this node to be assigned to.

    bool fixed;
    set<CircuitNode *> S;  // only for fixed node.

public:
    CircuitNode(string s) : Node(s) {
        fixed = false;
        fpga_node = nullptr;
    }
    void set_fixed(FPGANode *fn) {
        fixed = true;
        fpga_node = fn;
    }

    bool is_fixed() { return fixed; }
};



}  // namespace topart


#endif  // NODE_H_
