/* Copyright (C) 2023 Bo-Wei Chen<time.chenbw@gmail.com> - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#include <fstream>
#include <iostream>

#include "db.h"
#include "parse.h"

using namespace std;

int main(int argc, char *argv[]) {
    ios_base::sync_with_stdio(0);
    cin.tie(0);

    if (argc != 3) {
        cout << "********************** [Error] *******************" << endl;
        cout << "*  [Usage] ./topart <input file> <output file> *" << endl;
        cout << "********************** [Error] *******************" << endl;
        return 1;
    }

    fstream input(argv[1], std::fstream::in);
    fstream output(argv[2], std::fstream::out);

    topart::DB db;
    topart::parse_input(db, input);
    db.timer.output_time("FINISH Parsing");

    db.calculate_candidate_fpga();
    db.partition();
    db.timer.output_time("FINISH Preprocessing");

    db.refine();
    db.timer.output_time("ALL FINISH");
    db.output(output);

    input.close();
    output.close();
    return 0;
}
