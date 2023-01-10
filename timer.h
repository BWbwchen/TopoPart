#ifndef TIMER_H_
#define TIMER_H_

#include <chrono>
#include <iostream>
#include <string>

namespace topart {

using std::cout;
using std::endl;
using std::string;
using std::chrono::duration;
using std::chrono::high_resolution_clock;
using std::chrono::time_point;

const double TIME_LIMIT = 550;

class Timer {
public:
    time_point<high_resolution_clock> start;
    time_point<high_resolution_clock> now;

public:
    Timer(){};
    void set_start() { start = high_resolution_clock::now(); }
    void output_time(string s) {
        now = high_resolution_clock::now();
        double tmp_duration =
            duration<double, std::ratio<1, 1>>(now - start).count();
        cout << "[BW] " << s << " @ time " << tmp_duration << endl;
    }
    bool timeout() {
        now = high_resolution_clock::now();
        double tmp_duration =
            duration<double, std::ratio<1, 1>>(now - start).count();

        if (tmp_duration > TIME_LIMIT) {
            return true;
        }
        return false;
    }
};

}  // namespace topart
#endif  // TIMER_H_
