// Separate namespace prevents different generated inline constants from violating
// the C++ one-definition rule. This object always uses the changed CAD header.
namespace physical {
#include "parameters.hpp"
}
namespace p = physical::datum::robot::main;
double physical_tool_length() {
    return p::forearm_length_m + p::gripper_palm_length_m + p::finger_length_m / 2;
}
