"""Exercise lazy URDF bring-up with stub preflight/exporter executables."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'parameter_studio'))
from datum_parameter_studio.lazy_urdf import bring_up_urdf

# Stub preflight: scans the passed project directory for the assembly and
# reports a passing source check with the assembly as the single source.
PREFLIGHT = r'''#!/usr/bin/env python3
import json, os, sys
root, assembly = sys.argv[1], sys.argv[2]
exists = os.path.isfile(os.path.join(root, assembly))
print(json.dumps({
    'source_checks': 'pass' if exists else 'fail',
    'sources': [assembly] if exists else [],
    'source_fidelity': {'status': 'structural-only', 'coupling': False},
    'evidence': 'Source preflight failed (stub)',
}))
'''

STL = '''solid arm
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 1 0
 endloop
endfacet
facet normal 1 0 0
 outer loop
  vertex 0 0 0
  vertex 0 0 1
  vertex 0 1 0
 endloop
endfacet
endsolid arm
'''

STEP_CONTENT = 'ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((\'stub\'));;\nEND-ISO-10303-21;\n'

MINIMAL_URDF = '''<robot name="lazy">
  <link name="base_link"/>
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry><mesh filename="robot-arm.stl"/></geometry>
    </visual>
  </link>
  <joint name="base_to_arm" type="fixed">
    <parent link="base_link"/>
    <child link="arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
'''

# Writes one URDF+mesh per invocation, and any STEP output it was given.
EXPORTER = r'''#!/usr/bin/env python3
import pathlib, sys
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
for index, arg in enumerate(args):
    if arg != '--out':
        continue
    out = pathlib.Path(args[index + 1])
    if out.suffix == '.step':
        out.write_text(%r)
    else:
        out.write_text(%r)
        out.with_name('robot-arm.stl').write_text(%r)
''' % (STEP_CONTENT, MINIMAL_URDF, STL)

# Ignores the STEP output entirely and still succeeds.
EXPORTER_URDF_ONLY = r'''#!/usr/bin/env python3
import pathlib, sys
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
outs = [pathlib.Path(args[i + 1]) for i, arg in enumerate(args) if arg == '--out']
for out in outs:
    if out.suffix != '.step':
        out.write_text(%r)
        out.with_name('robot-arm.stl').write_text(%r)
''' % (MINIMAL_URDF, STL)

# Rejects the extra output (nonzero exit), but a URDF-only retry succeeds.
EXPORTER_REJECTS_STEP = r'''#!/usr/bin/env python3
import pathlib, sys, time
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
outs = [pathlib.Path(args[i + 1]) for i, arg in enumerate(args) if arg == '--out']
if len(outs) > 1:
    time.sleep(0.6)
    sys.exit(3)
time.sleep(0.4)
out = outs[0]
out.write_text(%r)
out.with_name('robot-arm.stl').write_text(%r)
''' % (MINIMAL_URDF, STL)

# Hangs on the multi-output invocation: a timeout must not be retried.
EXPORTER_HANGS = r'''#!/usr/bin/env python3
import pathlib, sys, time
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
outs = [pathlib.Path(args[i + 1]) for i, arg in enumerate(args) if arg == '--out']
if len(outs) > 1:
    time.sleep(5)
out = outs[0]
out.write_text(%r)
out.with_name('robot-arm.stl').write_text(%r)
''' % (MINIMAL_URDF, STL)

# Writes CRLF line endings to prove the reported hash is the file's bytes.
EXPORTER_CRLF = r'''#!/usr/bin/env python3
import pathlib, sys
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
for index, arg in enumerate(args):
    if arg != '--out':
        continue
    out = pathlib.Path(args[index + 1])
    if out.suffix == '.step':
        out.write_text(%r)
    else:
        out.write_bytes(%r)
        out.with_name('robot-arm.stl').write_text(%r)
''' % (STEP_CONTENT, MINIMAL_URDF.replace('\n', '\r\n').encode(), STL)

# Writes a STEP output that is not STEP text.
EXPORTER_BAD_STEP = r'''#!/usr/bin/env python3
import pathlib, sys
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
for index, arg in enumerate(args):
    if arg != '--out':
        continue
    out = pathlib.Path(args[index + 1])
    if out.suffix == '.step':
        out.write_text('not a STEP file\n')
    else:
        out.write_text(%r)
        out.with_name('robot-arm.stl').write_text(%r)
''' % (MINIMAL_URDF, STL)

# Attempts to redirect bundle writes outside the bundle via symlinks.
EXPORTER_SYMLINKS = r'''#!/usr/bin/env python3
import pathlib, sys
args = sys.argv
if '--version' in args:
    print('stub-exporter 1')
    sys.exit(0)
import os
outs = [pathlib.Path(args[i + 1]) for i, arg in enumerate(args) if arg == '--out']
bundle = outs[0].parent
outside = pathlib.Path(os.environ['STUB_OUTSIDE'])
(outside / 'steps').mkdir(parents=True, exist_ok=True)
(outside / 'victim.txt').write_text('original')
(bundle / 'steps').symlink_to(outside / 'steps', target_is_directory=True)
(bundle / 'report.json').symlink_to(outside / 'victim.txt')
(bundle / 'assembly.step').symlink_to(outside / 'step.step')
outs[0].write_text(%r)
outs[0].with_name('robot-arm.stl').write_text(%r)
''' % (MINIMAL_URDF, STL)


def write_stub(path, content):
    path.write_text(content)
    path.chmod(path.stat().st_mode | 0o111)


def write_source_with_blobs(path):
    """A real ZIP-shaped source with content-addressed STEP blobs.

    One blob is named after its own SHA-256 (kept); one is named after a
    different content (skipped with a diagnostic).
    """
    good = hashlib.sha256(STEP_CONTENT.encode()).hexdigest()
    decoy_content = 'ISO-10303-21;\nnot the named content\n'
    decoy_name = hashlib.sha256(b'other').hexdigest()
    entries = {
        'container.json': '{}',
        'document.json': '{}',
        'blobs/%s.step' % good: STEP_CONTENT,
        'blobs/%s.step' % decoy_name: decoy_content,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, 'w') as archive:
        for name, content in entries.items():
            archive.writestr(name, content)
    return good, decoy_name


class LazyUrdfTest(unittest.TestCase):
    def make_project(self, root, assembly='cad/robot.solidcraft', blobs=False):
        target = root / assembly
        if blobs:
            good, decoy = write_source_with_blobs(target)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(b'stub-solidcraft-file')
            good = decoy = None
        return good, decoy

    def stubs(self, root, exporter_source=EXPORTER):
        preflight = root / 'preflight'
        exporter = root / 'exporter'
        write_stub(preflight, PREFLIGHT)
        write_stub(exporter, exporter_source)
        return preflight, exporter

    def test_bring_up_exports_checks_and_reports(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root)
            self.make_project(root)
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter) as result:
                self.assertEqual('<robot name="lazy">', result.text.splitlines()[0])
                self.assertEqual(result.sha256, hashlib.sha256(result.text.encode()).hexdigest())
                report = result.report
                self.assertEqual(report['execution'], 'complete')
                self.assertEqual(report['structural_checks']['status'], 'pass')
                self.assertEqual(report['freshness'], 'current')
                self.assertEqual(report['capture']['mode'], 'lazy saved-files')
                self.assertEqual(report['schema'], 'datum.lazy-urdf-report/3')
                self.assertEqual(report['outputs_sha256']['robot.urdf'], hashlib.sha256(result.text.encode()).hexdigest())
                # Geometry lives in the bundle and is reported with hashes.
                geometry = report['geometry']
                self.assertEqual(list(geometry['meshes']), ['robot-arm.stl'])
                self.assertEqual(geometry['mesh_links']['robot-arm.stl'], ['arm'])
                self.assertEqual(geometry['meshes']['robot-arm.stl'], hashlib.sha256(STL.encode()).hexdigest())
                self.assertEqual(geometry['assembly_step']['path'], 'assembly.step')
                self.assertEqual(geometry['assembly_step']['sha256'], hashlib.sha256(STEP_CONTENT.encode()).hexdigest())
                self.assertIn('hash-verified', geometry['note'])
                self.assertIn('blob-to-URDF-link correspondence is not established', geometry['note'])
                # report.json is written but excluded from the hash inventory.
                self.assertIn('report.json', [path.name for path in result.bundle.iterdir()])
                self.assertNotIn('report.json', report['outputs_sha256'])

    def test_bundle_carries_meshes_and_can_be_copied(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root)
            self.make_project(root)
            destination = root / 'delivered'
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter) as result:
                result.write_bundle(destination)
                self.assertTrue((destination / 'robot.urdf').is_file())
                self.assertTrue((destination / 'robot-arm.stl').is_file())
                self.assertIn('robot-arm.stl', (destination / 'robot.urdf').read_text())
                with self.assertRaises(FileExistsError):
                    result.write_bundle(destination)
            self.assertFalse(result.bundle.exists())

    def test_part_step_blobs_are_extracted_with_verified_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root)
            good, decoy = self.make_project(root, blobs=True)
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter) as result:
                geometry = result.report['geometry']
                self.assertEqual(geometry['part_steps']['blobs'], {good: 'cad/robot.solidcraft'})
                diagnostics = geometry['part_steps']['diagnostics']
                self.assertTrue(any(decoy in entry['reason'] for entry in diagnostics))
                self.assertEqual(set(result.report['outputs_sha256']), {'robot.urdf', 'robot-arm.stl', 'assembly.step', 'steps/%s.step' % good})
                self.assertTrue((result.bundle / 'steps' / ('%s.step' % good)).is_file())

    def test_exporter_without_step_output_degrades_to_urdf_and_meshes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root, EXPORTER_URDF_ONLY)
            self.make_project(root, blobs=True)
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter) as result:
                report = result.report
                self.assertEqual(report['structural_checks']['status'], 'pass')
                geometry = report['geometry']
                self.assertIsNone(geometry['assembly_step'])
                self.assertIn('wrote no assembly STEP', geometry['unavailable_reason'])
                self.assertEqual(list(geometry['meshes']), ['robot-arm.stl'])

    def test_exporter_rejecting_extra_output_is_retried_with_fresh_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root, EXPORTER_REJECTS_STEP)
            self.make_project(root)
            # The rejected attempt sleeps 0.6 s and the retry 0.4 s; a reused
            # one-second budget would make the retry fail.
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter, timeout=1) as result:
                geometry = result.report['geometry']
                self.assertIsNone(geometry['assembly_step'])
                self.assertIn('rejected the extra assembly STEP', geometry['unavailable_reason'])
                self.assertEqual(result.report['structural_checks']['status'], 'pass')

    def test_hanging_exporter_times_out_without_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root, EXPORTER_HANGS)
            self.make_project(root)
            with self.assertRaises(TimeoutError):
                bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter, timeout=1)

    def test_hash_covers_exact_urdf_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root, EXPORTER_CRLF)
            self.make_project(root)
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter) as result:
                self.assertIn('\r\n', result.text)
                self.assertEqual(result.sha256, result.report['outputs_sha256']['robot.urdf'])
                delivered = root / 'delivered.urdf'
                result.write(delivered)
                self.assertEqual(hashlib.sha256(delivered.read_bytes()).hexdigest(), result.sha256)

    def test_non_step_assembly_output_is_not_advertised(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root, EXPORTER_BAD_STEP)
            self.make_project(root)
            with bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter) as result:
                geometry = result.report['geometry']
                self.assertIsNone(geometry['assembly_step'])
                self.assertIn('not STEP text', geometry['unavailable_reason'])
                self.assertFalse((result.bundle / 'assembly.step').exists())

    def test_symlinked_exporter_output_is_rejected_without_writing_outside(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root, EXPORTER_SYMLINKS)
            self.make_project(root)
            outside = root / 'outside'
            os.environ['STUB_OUTSIDE'] = str(outside)
            try:
                with self.assertRaisesRegex(ValueError, 'symlink'):
                    bring_up_urdf(root, 'cad/robot.solidcraft', preflight=preflight, exporter=exporter)
            finally:
                del os.environ['STUB_OUTSIDE']
            self.assertEqual((outside / 'victim.txt').read_text(), 'original')
            self.assertEqual(list((outside / 'steps').iterdir()), [])

    def test_missing_assembly_fails_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, _ = self.stubs(root)
            with self.assertRaisesRegex(ValueError, 'Source preflight failed'):
                bring_up_urdf(root, 'cad/absent.solidcraft', preflight=preflight, exporter=root / 'exporter')

    def test_cli_prints_urdf_and_writes_evidence_and_bundle(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            preflight, exporter = self.stubs(root)
            self.make_project(root)
            evidence = root / 'evidence/urdf-report.json'
            bundle = root / 'cli-bundle'
            result = subprocess.run(
                [sys.executable, '-m', 'datum_parameter_studio.lazy_urdf',
                 '--project', str(root), '--assembly', 'cad/robot.solidcraft',
                 '--preflight', str(preflight), '--exporter', str(exporter),
                 '--evidence', str(evidence), '--bundle-out', str(bundle)],
                capture_output=True, text=True,
                env={'PYTHONPATH': str(ROOT / 'parameter_studio'), 'PATH': '/usr/bin:/bin'},
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.splitlines()[0], '<robot name="lazy">')
            report = json.loads(result.stderr.splitlines()[-1])
            self.assertEqual(report['schema'], 'datum.lazy-urdf-report/3')
            saved = json.loads(evidence.read_text())
            self.assertEqual(saved['outputs_sha256']['robot.urdf'], report['outputs_sha256']['robot.urdf'])
            self.assertEqual(saved['geometry']['meshes'], report['geometry']['meshes'])
            self.assertTrue((bundle / 'robot-arm.stl').is_file())
            self.assertTrue((bundle / 'report.json').is_file())


if __name__ == '__main__':
    unittest.main()
