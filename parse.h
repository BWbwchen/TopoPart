#ifndef PARSE_H_
#define PARSE_H_


#include <fstream>

#include "db.h"

using std::fstream;

namespace topart {

void parse_input(DB &db, fstream &input);

}  // namespace topart

#endif  // PARSE_H_
