"""Worktree-aware Datum CAD parameters, usable from Python with optional ROS helpers."""
import hashlib
import json
import math
from pathlib import Path


class Parameters:
    """Immutable-style named values loaded from a checked Datum manifest."""
    def __init__(self, manifest):
        self.manifest = manifest
        self.worktree = manifest["worktree"]
        self.values = manifest["parameters"]
        self.groups = manifest["groups"]

    def __getitem__(self, name):
        return self.values[name]

    def get(self, name, default=None):
        return self.values.get(name, default)

    def group(self, name):
        """Return named values for one mapped CAD feature group."""
        return {key: self.values[key] for key in self.groups[name]}

    def ros_parameters(self):
        """Return a flat dict accepted by rclpy Node(parameters=[...])."""
        return dict(self.values)

    def ros_yaml(self, node_name):
        """Return a node-scoped mapping suitable for ROS parameter YAML."""
        if not node_name or not isinstance(node_name, str):
            raise ValueError("node_name must be a non-empty string")
        return {node_name: {"ros__parameters": dict(self.values)}}


from .lazy import evaluate_lazy, write_evidence


def _check_manifest(manifest):
    """Shared value contract for compiled and lazy manifests.

    Both loaders must accept and reject the same values: a manifest with a
    non-positive parameter outside its bounds, a non-finite value or malformed
    groups is invalid however it was produced.
    """
    if manifest.get("schema") != "datum.robot.parameters/2" or manifest.get("units") != "m":
        raise ValueError("unsupported Datum parameter manifest")
    values = manifest.get("parameters")
    if not isinstance(values, dict) or not values:
        raise ValueError("Datum manifest has no parameters")
    bounds = manifest.get("bounds", {})
    if not isinstance(bounds, dict):
        raise ValueError("invalid Datum parameter bounds")
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"invalid Datum parameter {name}")
        limits = bounds.get(name, {})
        if not isinstance(limits, dict):
            raise ValueError(f"invalid Datum parameter bounds for {name}")
        if value <= 0 and limits.get("allow_nonpositive") is not True:
            raise ValueError(f"invalid non-positive Datum parameter {name}")
        if value < (limits.get("min") if limits.get("min") is not None else -math.inf) or value > (limits.get("max") if limits.get("max") is not None else math.inf):
            raise ValueError(f"Datum parameter {name} is outside its allowed bounds")
    groups = manifest.get("groups")
    if not isinstance(groups, dict) or any(not isinstance(v, list) or any(n not in values for n in v) for v in groups.values()):
        raise ValueError("invalid Datum feature groups")
    return manifest


def load_lazy(project, *, worktree="main", evidence=None):
    """Load parameters directly from spec and saved CAD, with no generated files.

    Evaluates PROJECT/datum/spec.json for the selected worktree against the
    saved SolidCraft sources and returns the same Parameters object as load().
    No generated parameters.json or parameters.hpp needs to exist. The manifest
    passes the same value contract as load(): missing fields, unit mismatches,
    out-of-bounds values, non-positive dimensions and malformed groups raise
    ValueError instead of returning values. Values are re-read at startup;
    concurrent edits during evaluation are detected by the source-hash check,
    not blocked. When evidence is a path, a machine-readable record of this
    evaluation (evidence schema, mode, values, bounds, source and spec SHA-256
    hashes) is written there.
    """
    manifest, lazy_evidence = evaluate_lazy(project, worktree=worktree)
    _check_manifest(manifest)
    if evidence is not None:
        write_evidence(evidence, lazy_evidence)
    return Parameters(manifest)


def load(manifest_path, *, worktree=None):
    """Load a generated manifest after checking the selected tree and CAD hashes."""
    path = Path(manifest_path).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "datum.robot.parameters/2" or manifest.get("units") != "m":
        raise ValueError("unsupported Datum parameter manifest")
    selected = worktree if worktree is not None else manifest.get("worktree")
    if selected != manifest.get("worktree"):
        raise ValueError(f"manifest is for worktree {manifest.get('worktree')!r}, not {selected!r}")
    spec = path.with_name("spec.json")
    if not spec.is_file() or hashlib.sha256(spec.read_bytes()).hexdigest() != manifest.get("spec_sha256"):
        raise ValueError("Datum parameter specification changed; regenerate parameters and rebuild")
    root = path.parent.parent.resolve()
    sources = manifest.get("sources_sha256")
    if not isinstance(sources, dict) or not sources:
        raise ValueError("Datum manifest has no CAD source hashes")
    for name, expected in sources.items():
        candidate = (root / name).resolve(strict=True)
        if not candidate.is_relative_to(root):
            raise ValueError(f"Datum CAD source escapes project: {name}")
        if hashlib.sha256(candidate.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Datum CAD source changed: {name}; regenerate parameters and rebuild")
    _check_manifest(manifest)
    return Parameters(manifest)
