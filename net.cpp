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



}  // namespace topart
