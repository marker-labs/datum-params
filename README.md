# Datum Parameters

DParams extracts explicit SolidCraft dimensions into a hash-checked JSON manifest and C++ header. The local Studio provides a mobile-friendly CAD viewer, parameter picker, and experimental URDF export/checking. It works on saved files without a Datum repository, account or cloud service.

## Install

Requires Python 3.10+, Rust/Cargo, Docker Compose and a host installation of `solidcraft-cli` for CAD rendering and URDF export. From a clone:

```sh
git clone https://github.com/marker-labs/datum-params.git
cd datum-params
python3 -m venv .venv
. .venv/bin/activate
python -m pip install ./datum_interface ./parameter_studio
cargo build --manifest-path rust/Cargo.toml --bins
cargo install --path rust --bin datum-params --locked
dparam-std up --source-repo "$PWD" --project /path/to/your/cad/project
```

Open http://localhost:8000. For a phone on your Tailscale network add `--tailscale` and use the URL printed by the launcher. Studio runs in the foreground; use another terminal for other commands. See [Studio instructions](parameter_studio/README.md).

The runtime API has two modes. Compiled mode loads a generated file at `PROJECT/datum/parameters.json`, alongside `PROJECT/datum/spec.json`:

```python
from datum_interface import load
params = load('PROJECT/datum/parameters.json', worktree='main')
print(params['forearm_length_m'])
```

It verifies the spec and saved CAD hashes: keep the exact downloaded spec beside the manifest. Lazy mode gives the same Parameters object without any generated artifacts by re-reading the spec and saved CAD at startup. A failed evaluation raises instead of returning values, so a run cannot fall back to an earlier evaluation; concurrent edits during evaluation are detected by the source-hash check, not blocked, and a process keeps the values it read at startup:

```python
from datum_interface import load_lazy
params = load_lazy('PROJECT', worktree='main')            # no parameters.json needed
params = load_lazy('PROJECT', evidence='evidence/lazy.json')  # machine-readable record
```

The C++ side has the same choice: link the compiled constants from a generated `parameters.hpp`, or include `rust/include/dparams_lazy.h` and link the `libdatum_params_tools` static library and read the values at startup. The Rust generator and C ABI share one evaluation core; the Python loader is a separate implementation kept in step by cross-mode conformance tests. Snapshot-based generation belongs to the optional Datum integration and is not implemented in this standalone package.

## URDF export modes

The URDF exists only because most robotics and simulation systems cannot load the saved CAD natively. Studio exports it on demand from the saved sources (**Generate and check**); the result is written to a scratch directory you download, never to the project. For a runtime bring-up without the checked-in file, `datum_parameter_studio.lazy_urdf` runs the same capture preflight and structural checker in-process, validates the output bundle for symlinks, containment and size, and hands back the URDF text plus its export bundle:

```python
from datum_parameter_studio.lazy_urdf import bring_up_urdf
with bring_up_urdf('PROJECT', 'cad/arm_concept.solidcraft') as result:
    robot_description = result.text      # URDF text exported from the live sources
    result.write_bundle('runtime/robot') # URDF + meshes + STEP files, self-contained
```

Or stream it to stdout for a launch pipeline (add `--bundle-out DIR` for the geometry):

```sh
python -m datum_parameter_studio.lazy_urdf --project PROJECT --assembly cad/arm_concept.solidcraft
```

Export runs against the saved CAD at evaluation time and is stamped with the source hashes and a freshness result; there is no committed URDF file to go stale. The bundle holds the exporter's current output for the captured inputs: every URDF-referenced STL (the structural check requires those meshes to resolve inside the bundle), a whole-assembly STEP on request from the exporter, and the content-addressed per-part STEP blobs embedded in the saved files, extracted with a SHA-256 name verification before each write. The report's `geometry` section records all of it with hashes. Exporter provenance and fidelity to native CAD geometry are not verified, per-part STEP blobs are not mapped to URDF links, and this path has no cancellation channel or worker heartbeat. Export still requires the installed SolidCraft CLI and accepts the same limited SolidCraft/34 profile as the Studio spool; physics readiness and source-to-URDF equivalence are not verified.

## C++ example

The [ROS 2 C++ pick-and-place example](examples/ros2_cpp_pick_place/README.md) uses generated dimensions with an `rclcpp` controller and action-based motor simulator. The companion [standalone C++ comparison](examples/cpp-pick-place/README.md) contrasts stale and current dimensions against the same kinematic arm, in a compiled-header mode and a lazy mode that reads the CAD at startup with no generated files.

## Scope

The URDF preview/export requires a separately installed SolidCraft CLI. The initial validator accepts a limited SolidCraft/34 assembly profile; coupling, transform equivalence, physical mass properties, and simulation or hardware readiness are not certified. A structural pass is not a fidelity or safety guarantee. Consult the Studio guide before using a generated file.

Licensed under Apache-2.0. The SolidCraft executable is a separate dependency and is not included.
