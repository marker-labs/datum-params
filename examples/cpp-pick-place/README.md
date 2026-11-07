# C++ pick-and-place parameter example

This example uses generated C++ constants in a small pick-and-place **kinematic** controller. It needs neither ROS nor `rclpy`. It compares a deliberately stale forearm dimension with parameters generated from changed SolidCraft files, while both runs evaluate the same changed physical dimensions.

The model evaluates grasp, lift and place target positions using the arm's two-link inverse and forward kinematics. It does not simulate contact, a gripper, dynamics, collisions or an object being knocked over. Its 15 mm position tolerance is illustrative, not a hardware or safety limit.

## Run

Install Rust/Cargo, a C++17 compiler, `sha256sum` and SolidCraft with `solidcraft-cli` on PATH. From a checkout of this repository:

```sh
examples/cpp-pick-place/run.sh /tmp/dparams-cpp-pick-place
```

The destination must not exist. The script creates new native SolidCraft box files for baseline and changed dimensions, writes a parameter spec for each, invokes the real `datum-params` generator and compiles both C++ controllers. It compiles `physical.cpp` once against the changed header and links that **same object** into both cases. The controller source is the same for both executables; its include path selects the baseline or changed generated header. It does not edit source CAD or override hashes.

| Parameter | Baseline controller | Changed CAD/controller |
|---|---:|---:|
| Upper arm | 0.180 m | 0.180 m |
| Forearm | 0.100 m | 0.160 m |
| Palm | 0.050 m | 0.050 m |
| Finger | 0.055 m | 0.055 m |
| Effective tool: forearm + palm + finger / 2 | 0.1775 m | 0.2375 m |

The stale controller misses the fixed grasp target by 0.0600 m and stops. The generated controller reaches the grasp, lift and place positions within tolerance. Both use the same target coordinates and changed arm. Each program takes an expected result (`fail` or `pass`) and returns nonzero if the observed result differs, so the script is suitable for a reproducibility check.

## Lazy mode

`run.sh` also builds `pick_place_lazy`, a second consumer that needs **no generated artifacts at all**. It links the shared Rust evaluation core as a static library (`rust/include/dparams_lazy.h`) and extracts every dimension from the parameter spec and saved CAD at startup:

| Lazy run | Controller CAD | Physical CAD | Result |
|---|---|---|---|
| `pick_place_lazy changed changed pass [evidence.json]` | current | current | grasp, lift and place reached |
| `pick_place_lazy baseline changed fail [evidence.json]` | stale baseline | current | misses by 0.0600 m |

The same executable runs both scenarios; the only difference between them is which CAD directory it is pointed at. Nothing is baked in, so the values come from the CAD at startup and a failed evaluation clears the session instead of falling back to earlier numbers. That is a startup snapshot, not continuous tracking: after `dparams_lazy_eval` the process keeps the doubles it read, and changing the CAD or spec mid-run does not update them. Pointing a run at the wrong CAD directory is an explicit way to get stale values. When the optional evidence path is given, the machine-readable `datum.robot.lazy-evidence/1` record (values, bounds, source and spec SHA-256 hashes) is written for that exact controller evaluation, and the physical evaluation is recorded beside it as `EVIDENCE_PATH.physical.json`; a failed read still records the CAD it consumed.

The lazy loader mirrors the generator's runtime rules: units, bounds and positive dimensions. The Rust generator and the C ABI share one evaluation core; the Python loader is a separate implementation and the cross-mode conformance tests compare both modes on units, summation order, derived ordering and identifiers. An out-of-bounds or malformed CAD change stops the run instead of serving stale values. In C++ the plan loses its compile-time `constexpr` — the values live in a small struct after one startup evaluation, so hot loops still see plain doubles. The kinematics, targets, tolerance, stage output and exit codes are shared with the compiled controller (`plan.hpp`); an unreachable target is reported on stderr in both modes.

The output directory retains each saved CAD file, spec, source-hashed JSON manifest, generated C++ header, SHA-256 inventory, compiled executable and result text. This is a self-contained synthetic arm example; it does not claim equivalence to the exported URDF or the Bullet contact demonstration.

## Code boundary and freshness

`controller.cpp` includes the generated header and uses the named dimensions at the IK boundary. `physical.cpp` includes the changed header in a separate namespace and derives the physical tool length. `controller_lazy.cpp` reads the same dimensions from the spec and saved CAD at startup (see Lazy mode). If you change CAD or the spec, rerun the generator and rebuild the compiled consumers before running; the lazy consumer instead fails fast or picks up the new CAD at startup by design.

The generated C++ header contains constants; it **does not perform runtime CAD or spec hash checks**. The JSON manifest records those hashes, the separate Python `datum_interface` loader can validate them, and the lazy C++ core re-extracts from CAD so there is nothing to check against. This workflow generates both headers immediately before compiling; it is not runtime stale-data protection.

## Startup performance check

After running the example above, measure both modes on the **same changed CAD**:

```sh
python3 examples/cpp-pick-place/benchmark.py /tmp/dparams-cpp-pick-place/changed \
  --warmup 2 --runs 20 --output /tmp/dparams-startup-benchmark.json
```

The report records every wall-clock sample and its median, minimum and maximum. Each sample launches a fresh process and runs the complete kinematic plan; it includes process startup and execution, not just parameter loading. The lazy example reads the CAD twice (controller and physical model); the generated example uses compiled constants and performs no runtime freshness check. The script checks that both produce identical plan output and that the spec and CAD hashes remain unchanged throughout measurement. It alternates execution order after warmup; results depend on the machine, filesystem cache and system load.

Separate one-time measurements cover generating JSON/header artifacts and compiling the generated example, plus compiling the lazy example. They exclude building the Rust generator and library, CAD creation, and any ROS, URDF export, physics engine or simulator initialization. Thus these numbers estimate overhead in this small example, not full simulation startup; generated artifacts must be regenerated and rebuilt after CAD changes.
