#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $# != 1 || -e "$1" ]]; then echo 'Usage: run.sh NEW_OUTPUT_DIRECTORY' >&2; exit 2; fi
for tool in cargo solidcraft-cli c++ sha256sum; do command -v "$tool" >/dev/null || { echo "Missing $tool" >&2; exit 2; }; done
mkdir -p "$1"
out=$(cd "$1" && pwd)
cargo build --locked --manifest-path "$repo/rust/Cargo.toml" --bins --lib
for variant in baseline changed; do
  project="$out/$variant"
  mkdir -p "$project/cad" "$project/datum"
  for entry in upper_arm:180 forearm:100 gripper_palm:50 finger:55; do
    name=${entry%:*}; length=${entry#*:}
    if [[ $variant == changed && $name == forearm ]]; then length=160; fi
    printf '{"commands":[{"command":"solid.box","params":{"length":%s,"width":30,"height":30,"body_name":"%s"}}]}\n' "$length" "$name" > "$out/feature.json"
    solidcraft-cli run "$out/feature.json" --save "$project/cad/$name.solidcraft" --quiet
  done
  cat > "$project/datum/spec.json" <<'JSON'
{"schema":"datum.robot.parameter-spec/2","units":"m","worktrees":{"main":{"parameters":{
"upper_arm_length_m":{"path":"cad/upper_arm.solidcraft","pointer":"/features/0/length","source_units":"mm","unit":"m"},
"forearm_length_m":{"path":"cad/forearm.solidcraft","pointer":"/features/0/length","source_units":"mm","unit":"m"},
"gripper_palm_length_m":{"path":"cad/gripper_palm.solidcraft","pointer":"/features/0/length","source_units":"mm","unit":"m"},
"finger_length_m":{"path":"cad/finger.solidcraft","pointer":"/features/0/length","source_units":"mm","unit":"m"}},
"constants":{"shoulder_height_m":{"value":0.11,"unit":"m"}}}}}
JSON
  sha256sum "$project"/cad/*.solidcraft > "$project/datum/cad-sha256.txt"
  "$repo/rust/target/debug/datum-params" generate "$project" "$project/datum/spec.json" main "$project/datum/parameters.json" "$project/datum/parameters.hpp"
done
# The physical object file is compiled once from changed CAD and linked into both cases.
c++ -std=c++17 -Wall -Wextra -Werror -I "$out/changed/datum" -c "$repo/examples/cpp-pick-place/physical.cpp" -o "$out/physical.o"
for variant in baseline changed; do
  c++ -std=c++17 -Wall -Wextra -Werror -I "$out/$variant/datum" "$repo/examples/cpp-pick-place/controller.cpp" "$out/physical.o" -o "$out/$variant/pick_place"
done
# The lazy controller needs no generated artifacts at all; it links the shared
# evaluation core and reads the spec and saved CAD at startup.
c++ -std=c++17 -Wall -Wextra -Werror -I "$repo/rust/include" -I "$repo/examples/cpp-pick-place" \
  "$repo/examples/cpp-pick-place/controller_lazy.cpp" -o "$out/pick_place_lazy" \
  -L "$repo/rust/target/debug" -ldatum_params_tools -lpthread -ldl \
  -Wl,-rpath,"$repo/rust/target/debug"
printf '\nStale controller, changed physical arm:\n'
"$out/baseline/pick_place" fail | tee "$out/baseline/result.txt"
printf '\nGenerated controller, same changed physical arm:\n'
"$out/changed/pick_place" pass | tee "$out/changed/result.txt"
printf '\nLazy controller (no generated artifacts), current CAD directory on both sides:\n'
"$out/pick_place_lazy" "$out/changed" "$out/changed" pass "$out/changed/lazy-evidence.json" | tee "$out/changed/lazy-result.txt"
printf '\nSame lazy executable pointed at the stale baseline CAD directory:\n'
"$out/pick_place_lazy" "$out/baseline" "$out/changed" fail "$out/baseline/lazy-evidence.json" | tee "$out/baseline/lazy-result.txt"
printf '\nCAD, source-hashed manifests, generated headers, lazy evidence and results: %s\n' "$out"
