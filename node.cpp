/* Copyright (C) 2023 Bo-Wei Chen<time.chenbw@gmail.com> - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include "node.h"

namespace topart {


void CircuitNode::set_fixed(FPGANode *fn) {
    fixed = true;
    fpga_node = fn;

    // update Net
    for (auto &n : nets) {
        n->add_fpga(fn);
    }
}

void CircuitNode::add_fpga(FPGANode *fn) {
    fpga_node = fn;

    // update Net
    for (auto &n : nets) {
        n->add_fpga(fn);
    }
}

void CircuitNode::remove_fpga() {
    for (auto &n : nets) {
        n->remove_fpga(this->fpga_node);
    }

    fpga_node = nullptr;
}

void CircuitNode::flush_tsr_to_cddt(vector<FPGANode *> &mapping) {
    cddt.clear();
    for (intg i = 0; i < tsr_cddt.v.size(); ++i) {
        if (tsr_cddt.at(i) > 0)
            cddt.emplace(mapping[i]);
        else {
            tsr_cddt.at(i) = 0;
        }
    }
}

void CircuitNode::flush_cddt_to_tsr() {
    tsr_cddt.clear();
    for (auto &f : cddt) {
        tsr_cddt.at(f->name) += 1;
    }
}

void CircuitNode::calculate_cut_increment(
    FPGANode *if_f,
    unordered_set<CircuitNode *> neighbor_lists) {
    intg cut_increment = 0;

    for (auto &n : nets) {
        cut_increment +=
            n->net_cell.size() * n->estimate_increase_cut_size(if_f);
    }
    cut_increment_map[if_f->name] = cut_increment;
}

intg CircuitNode::try_move() {
    intg esti_dec = 0;
    for (auto &n : nets) {
        if (n->try_move(this->fpga_node))
            esti_dec++;
    }

    return esti_dec != 0;
}

intg CircuitNode::try_move(FPGANode *f) {
    intg dec = 0;
    for (auto &n : nets) {
        dec += n->estimate_increase_cut_size_refine(f);
    }

    return dec;
}



}  // namespace topart
