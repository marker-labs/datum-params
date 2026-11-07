"""Lazy parameter evaluation: value semantics shared with the compiled generator.

The Rust datum-params CLI bakes a manifest and C++ header before the build. This
module evaluates the same spec against the saved CAD files at runtime instead,
so no generated artifact is needed. Any mismatch (missing field, units, bounds,
positive-dimension rule) stops evaluation with a ValueError instead of running
stale.

The Rust generator and the C ABI share one evaluation core; this module is an
independent implementation of the same rules. It is kept in step by conformance
tests that compare both modes on units, summation order, derived ordering and
identifier rules, not by construction. Evaluated source hashes are re-read and
compared before returning, so a file that changes mid-evaluation fails instead
of producing values that do not match the recorded hash.
"""
import hashlib
import io
import json
import math
import os
from pathlib import Path
import zipfile

_SPEC_SCHEMA = "datum.robot.parameter-spec/2"
_UNIT_SCALE = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254, "ft": 0.3048}
_MAX_DOCUMENT_BYTES = 32 * 1024 * 1024


def _unit_scale(source, target):
    for unit in (source, target):
        if unit not in _UNIT_SCALE:
            raise ValueError("unsupported unit %r for CAD dimensions" % unit)
    return _UNIT_SCALE[source] / _UNIT_SCALE[target]


def _parse_float(text):
    """Parse like Rust's f64 FromStr for the numeric forms we accept.

    Python accepts digit separators such as ``1_0`` that Rust rejects.
    """
    if "_" in text:
        raise ValueError("unsupported numeric literal %r" % text)
    return float(text)


def _quantity(text, default_unit):
    text = text.strip()
    try:
        return _parse_float(text)
    except ValueError:
        pass
    parts = text.split()
    if len(parts) == 2:
        try:
            number = _parse_float(parts[0])
        except ValueError as error:
            raise ValueError("unsupported SolidCraft expression %r" % text) from error
        return number * _unit_scale(parts[1], default_unit)
    for unit in ("mm", "cm", "in", "ft", "m"):
        if not text.endswith(unit):
            continue
        try:
            number = _parse_float(text[: -len(unit)].strip())
        except ValueError:
            continue
        return number * _unit_scale(unit, default_unit)
    raise ValueError("unsupported SolidCraft expression %r; use a numeric literal with an optional unit" % text)


def _solidcraft_number(data, item, default_unit):
    if isinstance(item, bool):
        raise ValueError("CAD dimension is not numeric")
    if isinstance(item, (int, float)):
        return float(item)
    if not isinstance(item, str):
        raise ValueError("CAD dimension is not numeric")
    try:
        return _quantity(item, default_unit)
    except ValueError:
        pass
    params = data.get("params") if isinstance(data, dict) else None
    if isinstance(params, list):
        for parameter in params:
            if isinstance(parameter, dict) and parameter.get("name") == item:
                expression = parameter.get("expr")
                if not isinstance(expression, str):
                    raise ValueError("SolidCraft parameter %r has no expression" % item)
                unit = parameter.get("unit", default_unit)
                if not isinstance(unit, str):
                    raise ValueError("SolidCraft parameter %r has a non-string unit" % item)
                # The parameter's own unit is the expression's default; the
                # result is then converted from that unit to the document unit.
                return _quantity(expression, unit) * _unit_scale(unit, default_unit)
    raise ValueError("%r is not a numeric literal or a named SolidCraft parameter" % item)


def _safe_relative(root, name):
    if not isinstance(name, str) or not name or name.startswith('/') or '\\' in name:
        raise ValueError("invalid source path %r" % (name,))
    parts = name.split('/')
    if any(part in ('', '.', '..') for part in parts):
        raise ValueError("invalid source path %s" % name)
    path = Path(name)
    candidate = (root / path).resolve(strict=True)
    if not candidate.is_relative_to(root):
        raise ValueError("source escapes project: %s" % name)
    return candidate


def _array_index(token):
    """Array-index token rules matching serde_json's JSON pointer parser."""
    if not token or not token.isascii() or not token.isdigit():
        return None
    if len(token) > 1 and token[0] == "0":
        return None
    return int(token)


def _pointer_lookup(data, pointer, source):
    if not isinstance(pointer, str):
        raise ValueError("invalid JSON pointer for %s" % source)
    if pointer == "":
        return data
    if not pointer.startswith("/"):
        raise ValueError("%s has no feature/dimension at %r" % (source, pointer))
    item = data
    for raw_token in pointer.split("/")[1:]:
        token = raw_token.replace("~1", "/").replace("~0", "~")
        if isinstance(item, dict) and token in item:
            item = item[token]
        elif isinstance(item, list) and _array_index(token) is not None and _array_index(token) < len(item):
            item = item[_array_index(token)]
        else:
            raise ValueError("%s has no feature/dimension at %s" % (source, pointer))
    return item


def _number_or_none(value):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("bounds must be numeric")
    return float(value)


def _cad_value(root, mapping, hashes):
    if not isinstance(mapping, dict):
        raise ValueError("invalid CAD parameter mapping")
    for key in ("path", "pointer", "source_units", "unit"):
        if not isinstance(mapping.get(key), str) or not mapping[key]:
            raise ValueError("CAD parameter mapping is missing %s" % key)
    path = _safe_relative(root, mapping["path"])
    raw = path.read_bytes()
    try:
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            info = archive.getinfo("document.json")
            if info.file_size > _MAX_DOCUMENT_BYTES:
                raise ValueError("SolidCraft document too large")
            text = archive.read(info)
    except (zipfile.BadZipFile, KeyError) as error:
        raise ValueError("%s is not a SolidCraft archive" % mapping["path"]) from error
    try:
        data = json.loads(text)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise ValueError("invalid document.json in %s" % mapping["path"]) from error
    if not isinstance(data, dict):
        raise ValueError("invalid document.json in %s" % mapping["path"])
    if data.get("units") != mapping["source_units"]:
        raise ValueError("units mismatch: %s" % mapping["path"])
    item = _pointer_lookup(data, mapping["pointer"], mapping["path"])
    scale = _unit_scale(mapping["source_units"], mapping["unit"])
    value = _solidcraft_number(data, item, mapping["source_units"]) * scale
    minimum = _number_or_none(mapping.get("min"))
    maximum = _number_or_none(mapping.get("max"))
    minimum = minimum * scale if minimum is not None else None
    maximum = maximum * scale if maximum is not None else None
    _check_bounds("CAD dimension", value, minimum, maximum)
    digest = hashlib.sha256(raw).hexdigest()
    previous = hashes.get(mapping["path"])
    if previous is not None and previous != digest:
        raise ValueError("source changed during evaluation: %s" % mapping["path"])
    hashes[mapping["path"]] = digest
    return value


def _ident(name):
    return (
        isinstance(name, str)
        and bool(name)
        and not ("0" <= name[0] <= "9")
        and all(("a" <= ch <= "z") or ("0" <= ch <= "9") or ch == "_" for ch in name)
    )


def _check_bounds(name, value, minimum, maximum):
    finite = lambda x: x is not None and math.isfinite(x)
    if not math.isfinite(value) or (minimum is not None and not math.isfinite(minimum)) or (maximum is not None and not math.isfinite(maximum)) or minimum is not None and maximum is not None and minimum > maximum:
        raise ValueError("invalid value or bounds for %s" % name)
    if minimum is not None and finite(minimum) and value < minimum:
        raise ValueError("%s=%s is outside allowed bounds [%s, %s]" % (name, value, minimum, "+inf" if maximum is None else maximum))
    if maximum is not None and finite(maximum) and value > maximum:
        raise ValueError("%s=%s is outside allowed bounds [%s, %s]" % (name, value, "-inf" if minimum is None else minimum, maximum))


def _bounds_doc(minimum, maximum, allow_nonpositive=False):
    return {"min": minimum, "max": maximum, "allow_nonpositive": allow_nonpositive}


def _mapping(table, what):
    if not isinstance(table, dict):
        raise ValueError("invalid %s in the parameter specification" % what)
    return table


def _list(value, what):
    if value is None:
        return []
    if not isinstance(value, list):
        raise ValueError("invalid %s in the parameter specification" % what)
    return value


def evaluate_lazy(project_root, *, worktree="main"):
    """Evaluate the parameter spec against the saved CAD files.

    Reads PROJECT/datum/spec.json from the resolved project root and extracts
    every mapped dimension fresh from the referenced SolidCraft sources.
    Returns (manifest, evidence); the manifest has the exact
    datum.robot.parameters/2 shape of the compiled generator, and the evidence
    record adds evidence_schema, mode=lazy and the spec path.
    """
    project = Path(project_root).resolve(strict=True)
    spec_path = (project / "datum/spec.json").resolve(strict=True)
    spec_bytes = spec_path.read_bytes()
    try:
        spec = json.loads(spec_bytes)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise ValueError("invalid parameter specification") from error
    if not isinstance(spec, dict):
        raise ValueError("invalid parameter specification")
    if spec.get("schema") != _SPEC_SCHEMA or spec.get("units") != "m":
        raise ValueError("expected datum.robot.parameter-spec/2 with canonical units m")
    if not _ident(worktree):
        raise ValueError("invalid worktree name")
    tree = _mapping(spec.get("worktrees"), "worktrees").get(worktree)
    if tree is None:
        raise ValueError("worktree %r is not defined in the spec" % worktree)
    tree = _mapping(tree, "worktree")
    values = {}
    bounds = {}
    hashes = {}
    constants = _mapping(tree.get("constants") or {}, "constants")
    for name in sorted(constants):
        constant = _mapping(constants[name], "constant %s" % name)
        if not _ident(name) or constant.get("unit") != "m":
            raise ValueError("invalid constant %s" % name)
        value = constant.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("invalid value or bounds for %s" % name)
        _check_bounds(name, float(value), constant.get("min"), constant.get("max"))
        if value <= 0 and not (constant.get("min") is not None and constant["min"] <= 0):
            raise ValueError("invalid non-positive Datum parameter %s" % name)
        values[name] = float(value)
        bounds[name] = _bounds_doc(constant.get("min"), constant.get("max"), constant.get("min") is not None and constant["min"] <= 0.0)
    parameters = _mapping(tree.get("parameters") or {}, "parameters")
    for name in sorted(parameters):
        mapping = parameters[name]
        if not _ident(name):
            raise ValueError("invalid parameter name %s" % name)
        value = _cad_value(project, mapping, hashes)
        if value <= 0.0:
            raise ValueError("CAD dimension %s must be positive" % name)
        scale = _unit_scale(mapping["source_units"], mapping["unit"])
        minimum = mapping["min"] * scale if mapping.get("min") is not None else None
        maximum = mapping["max"] * scale if mapping.get("max") is not None else None
        _check_bounds(name, value, minimum, maximum)
        values[name] = value
        bounds[name] = _bounds_doc(minimum, maximum)
    derived = _mapping(tree.get("derived") or {}, "derived parameters")
    # The Rust core stores derived parameters in a BTreeMap, so definitions are
    # evaluated in name order; mirror that instead of JSON insertion order.
    for name in sorted(derived):
        definition = _mapping(derived[name], "derived parameter %s" % name)
        if not _ident(name):
            raise ValueError("invalid parameter name %s" % name)
        inputs = _list(definition.get("inputs"), "derived inputs")
        if not inputs or any(not isinstance(input_name, str) or input_name not in values for input_name in inputs):
            raise ValueError("%s references a missing input" % name)
        arguments = [values[input_name] for input_name in inputs]
        operation = definition.get("operation")
        if operation == "sum":
            value = 0.0
            for argument in arguments:
                value += argument
        elif operation == "product":
            value = math.prod(arguments)
        elif operation == "min":
            value = min(arguments)
        elif operation == "max":
            value = max(arguments)
        elif operation == "difference" and len(arguments) == 2:
            value = arguments[0] - arguments[1]
        elif operation == "ratio" and len(arguments) == 2:
            numerator, denominator = arguments
            if denominator == 0:
                value = math.nan if numerator == 0 else math.copysign(math.inf, numerator)
            else:
                value = numerator / denominator
        else:
            raise ValueError("unsupported operation or arity for %s" % name)
        if definition.get("unit") != "m":
            raise ValueError("derived parameter %s must use canonical unit m" % name)
        _check_bounds(name, value, definition.get("min"), definition.get("max"))
        if value <= 0 and not (definition.get("min") is not None and definition["min"] <= 0):
            raise ValueError("invalid non-positive Datum parameter %s" % name)
        values[name] = value
        bounds[name] = _bounds_doc(definition.get("min"), definition.get("max"), definition.get("min") is not None and definition["min"] <= 0.0)
    for source in _list(tree.get("sources"), "worktree sources") + _list(spec.get("sources"), "sources"):
        if not isinstance(source, str) or not source:
            raise ValueError("invalid source path %r" % (source,))
        path = _safe_relative(project, source)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        previous = hashes.get(source)
        if previous is not None and previous != digest:
            raise ValueError("source changed during evaluation: %s" % source)
        hashes[source] = digest
    for source, previous in hashes.items():
        if hashlib.sha256(_safe_relative(project, source).read_bytes()).hexdigest() != previous:
            raise ValueError("source changed during evaluation: %s" % source)
    groups = _mapping(tree.get("groups") or {}, "feature groups")
    for group in sorted(groups):
        names = groups[group]
        if not _ident(group) or not isinstance(names, list) or not names or any(not isinstance(name, str) or name not in values for name in names):
            raise ValueError("invalid group %s: every referenced parameter must exist" % group)
    manifest = {
        "schema": "datum.robot.parameters/2",
        "units": spec.get("units"),
        "worktree": worktree,
        "parameters": values,
        "groups": groups,
        "bounds": bounds,
        "metadata": {key: spec[key] for key in spec if key not in ("schema", "units", "worktrees", "sources")},
        "sources_sha256": hashes,
        "spec_sha256": hashlib.sha256(spec_bytes).hexdigest(),
    }
    evidence = dict(manifest, evidence_schema="datum.robot.lazy-evidence/1", mode="lazy", spec_path=str(spec_path))
    return manifest, evidence


def write_evidence(path, evidence):
    """Atomically persist a lazy-evaluation evidence record."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".tmp")
    tmp.write_text(json.dumps(evidence, indent=2) + "\n")
    os.replace(tmp, target)
