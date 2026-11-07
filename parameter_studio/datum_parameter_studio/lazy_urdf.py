"""Lazy URDF bring-up: export from saved CAD at runtime, never from a stale file.

The URDF exists in most stacks only because simulation systems cannot load the
native CAD. This module brings it up on demand: it captures the exact saved
sources with the same preflight and structural checker the Studio spool uses,
exports with the SolidCraft CLI into an isolated scratch directory, checks the
output, and returns the URDF text plus a machine-readable report. No checked-in
URDF file is required or updated, so a node brings up the exporter's current
output for the captured sources and stamps it with their hashes.

Geometry comes from the same evaluation: the URDF-referenced STLs are exported
into the bundle (the structural check rejects a URDF whose meshes are missing),
a whole-assembly STEP is requested from the exporter, and content-addressed
per-part STEP blobs embedded in the saved files are extracted into the bundle
after verifying each blob's SHA-256 against its own entry name. The bundle is
validated for symlinks, containment and size before and after those writes, and
it stays available on the result (``result.bundle``, ``write_bundle``) because
the URDF's relative mesh references need the STL files beside them. This
verifies that the bundle contains the exporter's current output for the
captured inputs; exporter provenance and fidelity to native CAD geometry are the
exporter's contract, not a runtime guarantee. Per-part STEP blobs are not
mapped to URDF links; that correspondence is not established here.

Unlike the Studio spool worker this path has no cancellation channel or worker
heartbeat. It does enforce the worker's output quota and symlink/containment
checks.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import zipfile

from datum_parameter_studio.urdf_check import check_urdf

_REPORT_SCHEMA = 'datum.lazy-urdf-report/3'
_MAX_OUTPUTS = 4096
_MAX_OUTPUT_BYTES = 256 * 1024 * 1024
_MAX_PART_STEP_BYTES = 64 * 1024 * 1024
_OUTPUT_TAIL = 65536


def _resolve_executable(name, env_key, role):
    configured = os.environ.get(env_key)
    path = Path(configured or name)
    if not path.is_file():
        found = shutil.which(str(path))
        if found:
            path = Path(found)
    if not path.is_file():
        raise ValueError('missing %s executable: set %s or install %s' % (role, env_key, path))
    return path


def _run_exporter(child, deadline):
    """Run a child in its own process group and return (code, output tail)."""
    log = tempfile.TemporaryFile()
    try:
        process = subprocess.Popen(child, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while process.poll() is None:
            if time.monotonic() > deadline:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise TimeoutError('URDF export timed out')
            time.sleep(.05)
        log.seek(0, os.SEEK_END)
        size = log.tell()
        log.seek(max(0, size - _OUTPUT_TAIL))
        output = log.read(_OUTPUT_TAIL).decode(errors='replace')
    finally:
        log.close()
    return process.returncode, output


def _regular_file(path, bundle):
    """True only for a non-symlink regular file that resolves inside the bundle."""
    return path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(bundle.resolve())


def _bundle_files(bundle):
    """Validate the exporter/extraction output and return its regular files.

    Mirrors the Studio worker's hardening: reject symlinks and paths that
    resolve outside the bundle, and enforce the output quota.
    """
    root = bundle.resolve()
    files = []
    for path in bundle.rglob('*'):
        if path.is_symlink():
            raise ValueError('export wrote a symlink into the bundle: %s' % path.name)
        if path.is_file():
            if not path.resolve().is_relative_to(root):
                raise ValueError('export wrote outside the bundle: %s' % path.name)
            files.append(path)
    if len(files) > _MAX_OUTPUTS or sum(path.stat().st_size for path in files) > _MAX_OUTPUT_BYTES:
        raise ValueError('export output quota exceeded')
    return sorted(files, key=lambda path: path.relative_to(bundle).as_posix())


def _write_exclusive(path, payload):
    """Create a bundle file without following a symlink at its destination."""
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(path, flags, 0o644)
    with os.fdopen(descriptor, 'wb') as handle:
        handle.write(payload)


def _write_atomic(path, text):
    """Write a bundle file atomically; os.replace replaces a symlink itself."""
    descriptor, temporary = tempfile.mkstemp(dir=str(path.parent), prefix='.%s.' % path.name, suffix='.tmp')
    try:
        with os.fdopen(descriptor, 'w') as handle:
            handle.write(text)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _mesh_bundle_mapping(urdf_path):
    """Map each URDF-referenced mesh filename to the links that reference it.

    Meshes outside a <link> get an empty link list rather than disappearing.
    """
    root = ET.parse(urdf_path).getroot()
    mapping = {}
    for link in root.findall('link'):
        for mesh in link.iter('mesh'):
            mapping.setdefault(mesh.get('filename', ''), []).append(link.get('name'))
    for mesh in root.iter('mesh'):
        mapping.setdefault(mesh.get('filename', ''), [])
    return mapping


def _extract_part_steps(scratch, sources, steps_dir):
    """Extract embedded content-addressed STEP blobs from captured sources.

    SolidCraft assemblies store geometry as blobs/<sha256>.step; the entry
    name is the content hash, so extraction verifies each blob against it.
    Returns {blob_sha256: source_path} for the kept blobs and a list of
    {source, reason} diagnostics for skipped files or blobs.
    """
    extracted = {}
    diagnostics = []
    for name in sorted(sources):
        archive = scratch / name
        try:
            with zipfile.ZipFile(archive) as bundle:
                for entry in sorted(bundle.namelist()):
                    filename = entry.rsplit('/', 1)[-1]
                    stem = filename[:-5] if filename.endswith('.step') else None
                    if not entry.startswith('blobs/') or not stem or len(stem) != 64 or any(ch not in '0123456789abcdef' for ch in stem):
                        continue
                    if stem in extracted:
                        continue
                    info = bundle.getinfo(entry)
                    if info.file_size > _MAX_PART_STEP_BYTES:
                        diagnostics.append({'source': name, 'reason': 'blob exceeds size limit: %s' % filename})
                        continue
                    payload = bundle.read(info)
                    if hashlib.sha256(payload).hexdigest() != stem:
                        diagnostics.append({'source': name, 'reason': 'blob content does not match its %s entry name' % filename})
                        continue
                    if not payload.startswith(b'ISO-10303-21;'):
                        diagnostics.append({'source': name, 'reason': 'blob is not STEP text: %s' % filename})
                        continue
                    try:
                        _write_exclusive(steps_dir / ('%s.step' % stem), payload)
                    except FileExistsError:
                        pass
                    extracted[stem] = name
        except (zipfile.BadZipFile, KeyError, OSError) as error:
            diagnostics.append({'source': name, 'reason': 'not an extractable SolidCraft archive: %s' % error})
    return extracted, diagnostics


def bring_up_urdf(project, assembly, *, preflight=None, exporter=None, timeout=120):
    """Export the saved CAD assembly to URDF at runtime and check it.

    Returns LazyUrdf with .text (the URDF), .report (capture/export/check
    record incl. source hashes and freshness), .sha256 (SHA-256 of the exact
    URDF bytes) and .bundle (the temp directory holding the URDF, its STL
    meshes, the assembly STEP and the extracted part STEP blobs). The bundle is
    removed when the result is closed, used as a context manager, or garbage
    collected; copy it with .write_bundle() or .write() before then if the
    relative mesh references must keep resolving. preflight and exporter
    override the studio-urdf-preflight and SolidCraft CLI executables; they
    must accept the same CLI shapes.
    """
    project = Path(project).resolve(strict=True)
    if (not isinstance(assembly, str) or not assembly or assembly.startswith('/')
            or '\\' in assembly or any(part in ('', '.', '..') for part in assembly.split('/'))):
        raise ValueError('assembly must be a project-relative path without . or .. components')
    preflight = Path(preflight) if preflight else _resolve_executable('studio-urdf-preflight', 'DATUM_URDF_PREFLIGHT', 'preflight')
    exporter = Path(exporter) if exporter else _resolve_executable('solidcraft-cli', 'DATUM_URDF_EXPORTER', 'exporter')

    def scan(root):
        result = subprocess.run([str(preflight), str(root), assembly], capture_output=True, text=True, timeout=30)
        try:
            data = json.loads(result.stdout)
        except json.JSONDecodeError as error:
            raise ValueError('source preflight produced no JSON') from error
        if data.get('source_checks') != 'pass':
            raise ValueError(data.get('evidence', 'Source preflight failed'))
        return data

    holder = tempfile.TemporaryDirectory(prefix='datum-lazy-urdf-')
    try:
        scratch = Path(holder.name) / 'project'
        scratch.mkdir()
        before = scan(project)
        for name in before.get('sources', []):
            relative = Path(name)
            if relative.is_absolute() or any(part in ('', '.', '..') for part in relative.parts):
                raise ValueError('invalid dependency path')
            source = (project / relative).resolve(strict=True)
            if not source.is_relative_to(project) or not source.is_file():
                raise ValueError('dependency escapes project')
            destination = scratch / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        captured = scan(scratch)
        if before.get('sources') != captured.get('sources') or before.get('sources') != scan(project).get('sources'):
            raise ValueError('Source changed during capture')
        bundle = Path(holder.name) / 'bundle'
        bundle.mkdir()
        urdf = bundle / 'robot.urdf'
        assembly_step = bundle / 'assembly.step'
        # Exporters without the extra STEP output must still give a fresh URDF
        # with its meshes; retry that with its own budget and report the STEP as
        # unavailable. A hanging exporter is not retried.
        returncode, output = _run_exporter(
            [str(exporter), 'run', '--quiet', '--in', str(scratch / assembly), '--out', str(urdf), '--out', str(assembly_step)],
            time.monotonic() + timeout,
        )
        if returncode or not _regular_file(urdf, bundle):
            # A partial or symlinked first attempt must not be reused.
            urdf.unlink(missing_ok=True)
            assembly_step.unlink(missing_ok=True)
            returncode, output = _run_exporter(
                [str(exporter), 'run', '--quiet', '--in', str(scratch / assembly), '--out', str(urdf)],
                time.monotonic() + timeout,
            )
            if returncode or not _regular_file(urdf, bundle):
                raise RuntimeError('URDF export failed (%s): %s' % (returncode, output))
            assembly_step.unlink(missing_ok=True)
            geometry_note = 'exporter rejected the extra assembly STEP output: %s' % output.strip()[-200:]
        elif not _regular_file(assembly_step, bundle):
            geometry_note = 'exporter wrote no assembly STEP output'
        else:
            geometry_note = ''
        _bundle_files(bundle)
        checks = check_urdf(urdf, bundle, synthetic_base='base_link')
        if checks['status'] != 'pass':
            raise ValueError('URDF structural checks failed: %s' % '; '.join(checks['findings']))
        mesh_links = _mesh_bundle_mapping(urdf)
        mesh_sha256 = {}
        for name in mesh_links:
            path = urdf.parent / name
            if path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(bundle):
                mesh_sha256[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        steps_dir = bundle / 'steps'
        steps_dir.mkdir()
        blobs, blob_diagnostics = _extract_part_steps(scratch, list(before.get('sources') or []), steps_dir)
        if not blobs:
            steps_dir.rmdir()
        if assembly_step.is_file():
            payload = assembly_step.read_bytes()
            if not payload.startswith(b'ISO-10303-21;'):
                assembly_step.unlink()
                geometry_note = geometry_note or 'exporter wrote an assembly STEP that is not STEP text'
        _bundle_files(bundle)
        try:
            fresh = scan(project).get('sources') == before.get('sources')
        except Exception:
            fresh = False
        version_code, version_output = _run_exporter([str(exporter), '--version'], time.monotonic() + 10)
        geometry = {
            'meshes': mesh_sha256,
            'mesh_links': mesh_links,
            'assembly_step': {
                'path': assembly_step.relative_to(bundle).as_posix(),
                'sha256': hashlib.sha256(assembly_step.read_bytes()).hexdigest(),
            } if assembly_step.is_file() else None,
            'part_steps': {'blobs': dict(blobs), 'diagnostics': blob_diagnostics},
            'note': ('Meshes and the whole-assembly STEP are the exporter\'s current output for the captured inputs and live in this bundle, '
                     'so a URDF cannot reference geometry that is absent from this evaluation. Content-addressed per-part STEP blobs were '
                     'extracted with hash-verified names; blob-to-URDF-link correspondence is not established and exporter fidelity to native '
                     'CAD is not verified here.'),
        }
        if geometry_note:
            geometry['unavailable_reason'] = geometry_note
        outputs = {path.relative_to(bundle).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in _bundle_files(bundle) if path.name != 'report.json'}
        report = {
            'schema': _REPORT_SCHEMA,
            'execution': 'complete',
            'capture': {'mode': 'lazy saved-files', 'assembly': assembly,
                        'source_hash_algorithm': 'blake3', 'output_hash_algorithm': 'sha256'},
            'exporter': {'version': version_output.strip() if version_code == 0 else None},
            'outputs_sha256': outputs,
            'geometry': geometry,
            'source_preflight': before,
            'structural_checks': checks,
            'assembly_fidelity': before.get('source_fidelity'),
            'freshness': 'current' if fresh else 'stale',
            'physics_readiness': 'unknown',
            'note': ('Exports from saved CAD at evaluation time; freshness was checked at export completion and the output bundle was '
                     'validated for symlinks, containment and quota. report.json is excluded from outputs_sha256. Physics and '
                     'source-to-URDF equivalence are not verified.'),
        }
        _write_atomic(bundle / 'report.json', json.dumps(report, indent=2) + '\n')
        return LazyUrdf(urdf.read_bytes(), report, holder, urdf, bundle)
    except Exception:
        holder.cleanup()
        raise


class LazyUrdf:
    """Runtime URDF bytes, report and export bundle; close() releases the bundle."""

    def __init__(self, data, report, holder, urdf_path, bundle):
        self._holder = holder
        self._data = data
        self.text = data.decode('utf-8')
        self.report = report
        self.sha256 = hashlib.sha256(data).hexdigest()
        self.urdf_path = urdf_path
        self.bundle = bundle

    def write(self, path):
        """Write the exact URDF bytes to a file of your choosing.

        Relative mesh references in the text only resolve while the bundle is
        alive; use write_bundle() for a self-contained copy.
        """
        Path(path).write_bytes(self._data)

    def write_bundle(self, destination):
        """Copy the URDF, meshes, STEP files and report into a new directory."""
        destination = Path(destination)
        if destination.exists():
            raise FileExistsError('bundle destination already exists: %s' % destination)
        shutil.copytree(self.bundle, destination)

    def close(self):
        """Remove the temporary export bundle (idempotent)."""
        holder, self._holder = self._holder, None
        if holder is not None:
            holder.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        self.close()
        return False

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def main():
    import argparse
    import sys
    parser = argparse.ArgumentParser(description='Print a lazily exported URDF to stdout')
    parser.add_argument('--project', required=True)
    parser.add_argument('--assembly', required=True)
    parser.add_argument('--preflight')
    parser.add_argument('--exporter')
    parser.add_argument('--timeout', type=int, default=120)
    parser.add_argument('--evidence', help='optional path for the machine-readable report')
    parser.add_argument('--bundle-out', help='optional directory to copy the URDF, meshes and STEP files into')
    args = parser.parse_args()
    try:
        result = bring_up_urdf(args.project, args.assembly, preflight=args.preflight, exporter=args.exporter, timeout=args.timeout)
    except Exception as error:
        print('%s' % error, file=sys.stderr)
        return 1
    try:
        sys.stderr.write(json.dumps(result.report) + '\n')
        if args.evidence:
            try:
                Path(args.evidence).parent.mkdir(parents=True, exist_ok=True)
                Path(args.evidence).write_text(json.dumps(result.report, indent=2) + '\n')
            except OSError as error:
                print('evidence write failed: %s' % error, file=sys.stderr)
                return 1
        if args.bundle_out:
            try:
                result.write_bundle(args.bundle_out)
            except OSError as error:
                print('bundle copy failed: %s' % error, file=sys.stderr)
                return 1
        sys.stdout.write(result.text)
        return 0
    finally:
        result.close()


if __name__ == '__main__':
    raise SystemExit(main())
