// Shared pick-and-place plan for both controller entry points: the compiled
// header maintains constexpr constants and the lazy controller supplies values
// extracted from the saved CAD at startup. Identical kinematics, tolerance,
// stage output and exit codes so both modes are directly comparable; only the
// diagnostics stream for an unreachable target differs from data output.
#pragma once
#include <cmath>
#include <iomanip>
#include <ostream>

struct ArmPlan { double upper_arm_length_m, shoulder_height_m, tool_length_m; };

struct Stage { const char* name; double x, z; };
const Stage STAGES[] = {{"grasp", .25, .185}, {"lift", .25, .235}, {"place", .24, .185}};

// Returns 1 when every stage is reached, 0 after a tolerance miss and 2 when a
// stage target lies outside controller reach (reported on `err`, not `out`).
inline int run_plan(const ArmPlan& plan, double actual_tool, double tolerance, std::ostream& out, std::ostream& err) {
    out << std::fixed << std::setprecision(4)
        << "controller tool=" << plan.tool_length_m << " m; physical tool=" << actual_tool << " m\n";
    bool success = true;
    for (const auto& stage : STAGES) {
        const double height = stage.z - plan.shoulder_height_m;
        const double cosine = (stage.x*stage.x + height*height - plan.upper_arm_length_m*plan.upper_arm_length_m - plan.tool_length_m*plan.tool_length_m)
            / (2 * plan.upper_arm_length_m * plan.tool_length_m);
        if (cosine < -1.0 || cosine > 1.0) {
            err << stage.name << " target outside controller reach\n";
            return 2;
        }
        const double elbow = -std::acos(cosine);
        const double shoulder = std::atan2(height, stage.x) - std::atan2(plan.tool_length_m * std::sin(elbow), plan.upper_arm_length_m + plan.tool_length_m * std::cos(elbow));
        // Apply the same commanded angles to the changed physical arm in both runs.
        const double tip_x = plan.upper_arm_length_m * std::cos(shoulder) + actual_tool * std::cos(shoulder + elbow);
        const double tip_z = plan.shoulder_height_m + plan.upper_arm_length_m * std::sin(shoulder) + actual_tool * std::sin(shoulder + elbow);
        const double error = std::hypot(tip_x - stage.x, tip_z - stage.z);
        out << stage.name << ": target=(" << stage.x << ", " << stage.z << ") m; tip=("
            << tip_x << ", " << tip_z << ") m; error=" << error << " m\n";
        if (error > tolerance) {
            success = false;
            out << "FAIL: " << stage.name << " misses the " << tolerance << " m position tolerance; remaining stages skipped\n";
            break;
        }
    }
    if (success) out << "PASS: grasp, lift and place positions reached in the kinematic model\n";
    return success ? 1 : 0;
}
