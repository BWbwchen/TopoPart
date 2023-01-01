#include "log.h"

namespace topart {

using std::cout;
using std::endl;
using std::string;

void log(string s) {
    string end = "\n";
    if (s.back() == '\n')
        end = "";

    cout << "[BW] " << s + end;
}

}  // namespace topart
