#include "log.h"

namespace topart {

using std::cout;
using std::endl;
using std::string;

void log(string s) {
#ifdef LOG
    string end = "\n";
    if (s.back() == '\n')
        end = "";

    cout << "[BW] " << s + end;
    cout << std::flush;
#endif
}

}  // namespace topart
