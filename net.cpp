#include "node.h"

namespace topart {

intg Net::estimate_increase_cut_size(FPGANode *if_f) {
    if (used_fpga_node.size() == 0) {
        return 0;
    } else {
        if (used_fpga_node.count(if_f) > 0) {
            return 0;
        } else {
            return 1;
        }
    }
}

intg Net::estimate_increase_cut_size_refine(FPGANode *if_f) {
    if (used_fpga_node.size() == 0) {
        return 0;
    } else {
        if (used_fpga_node.count(if_f) > 0) {
            if (used_fpga_node_count[if_f->name] == 1)
                return -1;
            else
                return 0;
        } else {
            return 1;
        }
    }
}

bool Net::try_move(FPGANode *f) {
    return used_fpga_node_count[f->name] <= 1;
}

}  // namespace topart
