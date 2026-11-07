// Lazy mode: the same pick-and-place plan, but no generated artifacts are
// needed before the run. Parameter values are extracted from the parameter
// spec and saved SolidCraft files at startup through the shared evaluation
// core's C ABI, so this one executable cannot fall back to values from an
// earlier evaluation: a failed evaluation clears the session and stops the run.
// No compile-time constants are involved.
#include "plan.hpp"
#include "dparams_lazy.h"
#include <iostream>
#include <string>
#include <vector>

namespace {
// Copies the last error out of the C ABI, sizing the buffer from its
// query call; a positive return is the required byte count.
std::string last_error() {
    const int needed = dparams_lazy_error(nullptr, 0);
    if (needed <= 0) return {};
    std::vector<char> buffer(static_cast<size_t>(needed));
    if (dparams_lazy_error(buffer.data(), buffer.size()) != 0) return {};
    return std::string(buffer.data());
}

// Releases the active session even when a read fails part way through.
struct SessionGuard {
    ~SessionGuard() { dparams_lazy_release(); }
};

struct LoadedArm { double upper_arm_length_m = 0, shoulder_height_m = 0, tool_length_m = 0; };

bool read_named(const char* name, double* out) {
    if (dparams_lazy_get(name, out) != 0) {
        std::cerr << "lazy read " << name << ": " << last_error() << "\n";
        return false;
    }
    return true;
}

// Evaluates PROJECT/datum/spec.json for worktree main once and reads the arm
// dimensions. When evidence_path is given, the machine-readable lazy evidence
// record for this exact evaluation is written there before release, including
// when a read fails, so a failed run still records which CAD it consumed.
bool load_arm(const char* project, LoadedArm& arm, bool need_shoulder, const char* evidence_path) {
    const std::string spec = std::string(project) + "/datum/spec.json";
    if (dparams_lazy_eval(project, spec.c_str(), "main") != 0) {
        std::cerr << "lazy evaluation failed for " << project << ": " << last_error() << "\n";
        return false;
    }
    SessionGuard guard;
    double forearm = 0, palm = 0, finger = 0;
    const bool ok = read_named("upper_arm_length_m", &arm.upper_arm_length_m)
        && read_named("forearm_length_m", &forearm)
        && read_named("gripper_palm_length_m", &palm)
        && read_named("finger_length_m", &finger)
        && (!need_shoulder || read_named("shoulder_height_m", &arm.shoulder_height_m));
    arm.tool_length_m = forearm + palm + finger / 2;
    if (evidence_path && dparams_lazy_evidence(evidence_path) != 0) {
        std::cerr << "lazy evidence write failed: " << last_error() << "\n";
        return false;
    }
    return ok;
}
} // namespace

// Usage: pick_place_lazy CONTROLLER_PROJECT PHYSICAL_PROJECT pass|fail [EVIDENCE_PATH]
// CONTROLLER_PROJECT and PHYSICAL_PROJECT normally name the same reviewed CAD
// directory. Pointing them at different directories shows what running
// against a stale CAD snapshot looks like, while the executable itself never
// changes between the two runs. With EVIDENCE_PATH, the controller evaluation
// is recorded there and the physical evaluation beside it as
// EVIDENCE_PATH.physical.json.
int main(int argc, char** argv) {
    if (argc < 4 || argc > 5
        || (std::string(argv[3]) != "pass" && std::string(argv[3]) != "fail")) {
        std::cerr << "Usage: pick_place_lazy CONTROLLER_PROJECT PHYSICAL_PROJECT pass|fail [EVIDENCE_PATH]\n";
        return 2;
    }
    const double tolerance = 0.015; // illustrative position tolerance, metres
    LoadedArm controller_arm, physical_arm;
    const std::string physical_evidence = argc == 5 ? std::string(argv[4]) + ".physical.json" : std::string();
    if (!load_arm(argv[1], controller_arm, true, argc == 5 ? argv[4] : nullptr)) return 2;
    if (!load_arm(argv[2], physical_arm, false, argc == 5 ? physical_evidence.c_str() : nullptr)) return 2;
    const ArmPlan plan{controller_arm.upper_arm_length_m, controller_arm.shoulder_height_m,
                       controller_arm.tool_length_m};
    const int reached = run_plan(plan, physical_arm.tool_length_m, tolerance, std::cout, std::cerr);
    if (reached == 2) return 2;
    const bool passed = (std::string(argv[3]) == "pass");
    return (passed ? reached == 1 : reached == 0) ? 0 : 1;
}
