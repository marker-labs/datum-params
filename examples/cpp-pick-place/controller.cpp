// Plan with one CAD-derived parameter set and evaluate against the changed arm.
// Compiled mode: constants arrive from parameters.hpp generated before the
// build; regenerate and rebuild after spec or CAD changes.
#include "parameters.hpp"
#include "plan.hpp"
#include <iostream>
#include <string>

namespace p = datum::robot::main;
extern double physical_tool_length();

int main(int argc, char** argv) {
    if (argc != 2 || (std::string(argv[1]) != "pass" && std::string(argv[1]) != "fail")) {
        std::cerr << "Usage: pick_place pass|fail (expected result)\n";
        return 2;
    }
    const double tolerance = 0.015; // illustrative position tolerance, metres
    const ArmPlan plan{p::upper_arm_length_m, p::shoulder_height_m,
                       p::forearm_length_m + p::gripper_palm_length_m + p::finger_length_m / 2};
    const int reached = run_plan(plan, physical_tool_length(), tolerance, std::cout, std::cerr);
    if (reached == 2) return 2;
    const bool passed = (std::string(argv[1]) == "pass");
    return (passed ? reached == 1 : reached == 0) ? 0 : 1;
}
