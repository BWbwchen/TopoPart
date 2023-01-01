#include "parse.h"

#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace topart {

using std::getline;
using std::make_pair;
using std::pair;
using std::stringstream;

void parse_input(DB &db, fstream &input) {
    intg total_fpga, total_circuit_node, total_fixed_node;
    intg fpga_capacity;
    intg fpga_connect_channel;
    intg num_net;
    input >> total_fpga >> fpga_connect_channel >> fpga_capacity >>
        total_circuit_node >> num_net >> total_fixed_node;

    // FPGA
    vector<pair<intg, intg>> g_fpga;
    intg v, w;
    for (intg i = 0; i < fpga_connect_channel; ++i) {
        input >> v >> w;
        g_fpga.emplace_back(make_pair(v, w));
    }
    db.build_fpga_graph(total_fpga, fpga_capacity, g_fpga);

    // NOTE: weird
    string s;
    getline(input, s);

    // Circuit
    vector<vector<intg>> g_circuit(total_circuit_node);
    for (int i = 0; i < num_net; ++i) {
        getline(input, s);
        stringstream ss(s);
        ss >> v;
        while (ss >> w) {
            g_circuit[v].emplace_back(w);
        }
    }
    db.build_circuit_graph(total_circuit_node, g_circuit);

    // Fixed Circuit
    vector<pair<intg, intg>> fixed_circuit_to_fpga;
    for (int i = 0; i < total_fixed_node; ++i) {
        input >> v >> w;
        fixed_circuit_to_fpga.emplace_back(make_pair(v, w));
    }
    db.set_fixed_circuit(fixed_circuit_to_fpga);
}



}  // namespace topart
