import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'datum_interface/python'))
from datum_interface import load, load_lazy

class StandaloneTest(unittest.TestCase):
    def test_generation_and_stale_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / 'cad').mkdir(); (root / 'datum').mkdir()
            cad = root / 'cad/part.solidcraft'
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':160}]}))
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}}}}}
            spec_path = root / 'datum/spec.json'; spec_path.write_text(json.dumps(spec))
            manifest = root / 'datum/parameters.json'; header = root / 'datum/parameters.hpp'
            subprocess.run(['cargo','run','--quiet','--offline','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(root),str(spec_path),'main',str(manifest),str(header)],check=True)
            self.assertAlmostEqual(load(manifest, worktree='main')['length_m'], .16)
            cad.write_bytes(cad.read_bytes()+b'changed')
            with self.assertRaisesRegex(ValueError, 'CAD source changed'): load(manifest, worktree='main')

    def test_lazy_evaluation_agrees_with_compiled_artifacts(self):
        """Compiled and lazy modes must evaluate one spec to identical values."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / 'cad').mkdir(); (root / 'datum').mkdir()
            cad = root / 'cad/part.solidcraft'
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'mm','params':[{'name':'arm_length','expr':'160 mm','unit':'mm'}],'features':[{'type':'box','length':'arm_length'}]}))
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}},
                'constants':{'shoulder_height_m':{'value':.11,'unit':'m'}},
                'derived':{'reach_m':{'operation':'sum','inputs':['length_m','shoulder_height_m'],'unit':'m'}},
                'groups':{'arm':['length_m','reach_m']}}}}
            spec_path = root / 'datum/spec.json'; spec_path.write_text(json.dumps(spec))
            manifest_path = root / 'datum/parameters.json'
            subprocess.run(['cargo','run','--quiet','--offline','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(root),str(spec_path),'main',str(manifest_path),str(root/'datum/parameters.hpp')],check=True)
            compiled = json.loads(manifest_path.read_text())
            params, evidence = None, None
            lazy = load_lazy(root, worktree='main', evidence=root/'evidence/lazy.json')
            self.assertEqual(compiled['parameters'], lazy.values)
            self.assertEqual(compiled['sources_sha256'], lazy.manifest['sources_sha256'])
            self.assertEqual(compiled['spec_sha256'], lazy.manifest['spec_sha256'])
            self.assertEqual(compiled['groups'], lazy.manifest['groups'])
            evidence = (root/'evidence/lazy.json').read_text()
            self.assertIn('"mode": "lazy"', evidence)
            self.assertIn('"evidence_schema": "datum.robot.lazy-evidence/1"', evidence)
            self.assertIn('"spec_sha256"', evidence)
            self.assertAlmostEqual(lazy['reach_m'], .16 + .11)
            self.assertEqual(lazy.group('arm'), {'length_m':0.16,'reach_m':0.27})
            # The named SolidCraft parameter resolves identically in both modes.
            self.assertAlmostEqual(lazy['length_m'], .16)

    def test_lazy_requires_only_spec_and_cad(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / 'cad').mkdir(); (root / 'datum').mkdir()
            cad = root / 'cad/part.solidcraft'
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':160}]}))
            spec_path = root / 'datum/spec.json'
            spec_path.write_text(json.dumps(
                {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m','min':100,'max':200}}}}}))
            params = load_lazy(root, evidence=root/'datum/lazy-evidence.json')
            self.assertAlmostEqual(params['length_m'], .16)
            self.assertEqual(params.manifest['bounds']['length_m'], {'min':0.1,'max':0.2,'allow_nonpositive':False})
            # Bounds and structure are enforced at runtime, not baked in: changing
            # the CAD yields the new value rather than serving a stale artifact,
            # and an out-of-bounds CAD change is rejected at startup.
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':180}]}))
            self.assertAlmostEqual(load_lazy(root)['length_m'], .18)
            # An out-of-bounds CAD change is rejected at startup.
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':40}]}))
            with self.assertRaisesRegex(ValueError, 'outside allowed bounds'):
                load_lazy(root)
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'inch','features':[{'type':'box','length':6}]}))
            with self.assertRaisesRegex(ValueError, 'units mismatch'):
                load_lazy(root)
            with zipfile.ZipFile(cad, 'w') as archive:
                archive.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':250}]}))
            with self.assertRaisesRegex(ValueError, 'outside allowed bounds'):
                load_lazy(root)


def write_cad(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, 'w') as archive:
        archive.writestr('document.json', json.dumps(document))


def generate(root, spec):
    """Write spec.json and run the real compiled generator."""
    (root / 'datum').mkdir(parents=True, exist_ok=True)
    spec_path = root / 'datum/spec.json'
    spec_path.write_text(json.dumps(spec))
    manifest = root / 'datum/parameters.json'
    subprocess.run(['cargo','run','--quiet','--offline','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(root),str(spec_path),'main',str(manifest),str(root/'datum/parameters.hpp')],check=True)
    return manifest


class ConformanceTest(unittest.TestCase):
    """Properties the compiled core and the Python mirror must agree on."""

    def test_units_summation_derived_order_and_identifiers(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_cad(root / 'cad/arm.solidcraft', {
                'units': 'm',
                'params': [{'name': 'arm_length', 'expr': '160 mm', 'unit': 'mm'}],
                'features': [{'type': 'box', 'length': 'arm_length'}],
            })
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'length_m':{'path':'cad/arm.solidcraft','pointer':'/features/0/length','source_units':'m','unit':'m'}},
                'constants':{
                    'c_big':{'value':1e16,'unit':'m'}, 'c_one':{'value':1,'unit':'m'},
                    'c_one_again':{'value':1,'unit':'m'}, 'c_negative':{'value':-1e16,'unit':'m','min':-1e16},
                    'c_three':{'value':3,'unit':'m'}},
                # Deliberately out of name order: the Rust core evaluates derived
                # parameters in sorted order, so z_extra must still see a_base.
                'derived':{
                    'z_extra':{'operation':'sum','inputs':['a_base','c_one'],'unit':'m'},
                    'a_base':{'operation':'sum','inputs':['c_one','c_three'],'unit':'m'},
                    'compensated':{'operation':'sum','inputs':['c_big','c_one','c_one_again','c_negative','c_three'],'unit':'m'}},
            }}}
            manifest = json.loads(generate(root, spec).read_text())
            lazy = load_lazy(root)
            self.assertEqual(manifest['parameters'], lazy.values)
            # A parameter's own unit is the expression default, converted once
            # from the document unit: 160 mm is 0.16 m, not 0.00016 m.
            self.assertAlmostEqual(lazy['length_m'], 0.16)
            self.assertAlmostEqual(lazy['a_base'], 4.0)
            self.assertAlmostEqual(lazy['z_extra'], 5.0)
            # Left-to-right summation: 1e16 + 1 + 1 - 1e16 + 3 == 3, whereas
            # compensated summation (Python 3.12+ sum()) yields 5.
            self.assertAlmostEqual(lazy['compensated'], 3.0)

    def test_identifiers_reject_leading_digits_in_both_modes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_cad(root / 'cad/arm.solidcraft', {'units':'mm','features':[{'type':'box','length':160}]})
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'9bad':{'path':'cad/arm.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}}}}}
            (root / 'datum').mkdir(parents=True)
            spec_path = root / 'datum/spec.json'
            spec_path.write_text(json.dumps(spec))
            compiled = subprocess.run(['cargo','run','--quiet','--offline','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(root),str(spec_path),'main',str(root/'datum/parameters.json'),str(root/'datum/parameters.hpp')],capture_output=True,text=True)
            self.assertNotEqual(compiled.returncode, 0)
            self.assertIn('invalid parameter name 9bad', compiled.stderr)
            with self.assertRaisesRegex(ValueError, 'invalid parameter name 9bad'):
                load_lazy(root)

    def test_load_and_load_lazy_share_the_value_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_cad(root / 'cad/part.solidcraft', {'units':'mm','features':[{'type':'box','length':160}]})
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}},
                'constants':{'a':{'value':0.1,'unit':'m'},'b':{'value':0.255,'unit':'m'}},
                'derived':{'signed_offset':{'operation':'difference','inputs':['a','b'],'unit':'m'}}}}}
            (root / 'datum').mkdir(parents=True)
            spec_path = root / 'datum/spec.json'
            spec_path.write_text(json.dumps(spec))
            compiled = subprocess.run(['cargo','run','--quiet','--offline','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(root),str(spec_path),'main',str(root/'datum/parameters.json'),str(root/'datum/parameters.hpp')], capture_output=True, text=True)
            self.assertNotEqual(compiled.returncode, 0)
            self.assertIn('invalid non-positive Datum parameter signed_offset', compiled.stderr)
            with self.assertRaisesRegex(ValueError, 'invalid non-positive Datum parameter signed_offset'):
                load_lazy(root)
            spec['worktrees']['main']['derived']['signed_offset']['min'] = -1
            manifest = generate(root, spec)
            self.assertAlmostEqual(load(manifest).values['signed_offset'], -0.155)
            self.assertAlmostEqual(load_lazy(root).values['signed_offset'], -0.155)
            # An empty parameter set is rejected by both loaders.
            empty = json.loads(manifest.read_text())
            empty['parameters'] = {}
            empty_path = root / 'datum/empty.json'
            empty_path.write_text(json.dumps(empty))
            with self.assertRaisesRegex(ValueError, 'no parameters'):
                load(empty_path, worktree='main')
            spec_empty = dict(spec, worktrees={'main':{'parameters':{}}})
            (root / 'datum/spec.json').write_text(json.dumps(spec_empty))
            with self.assertRaisesRegex(ValueError, 'no parameters'):
                load_lazy(root)

    def test_malformed_inputs_raise_value_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'datum').mkdir(parents=True)
            # Non-object document.json.
            write_cad(root / 'cad/part.solidcraft', [])
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}}}}}
            (root / 'datum/spec.json').write_text(json.dumps(spec))
            with self.assertRaisesRegex(ValueError, 'invalid document.json'):
                load_lazy(root)
            # Missing required mapping field.
            write_cad(root / 'cad/part.solidcraft', {'units':'mm','features':[{'type':'box','length':160}]})
            broken = json.loads(json.dumps(spec))
            del broken['worktrees']['main']['parameters']['length_m']['pointer']
            (root / 'datum/spec.json').write_text(json.dumps(broken))
            with self.assertRaisesRegex(ValueError, 'missing pointer'):
                load_lazy(root)
            # Zero-divisor ratio becomes an invalid value, not a raw ZeroDivisionError.
            zero = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}},
                'constants':{'zero':{'value':0,'unit':'m','min':0}},
                'derived':{'bad_ratio':{'operation':'ratio','inputs':['zero','zero'],'unit':'m'}}}}}
            (root / 'datum/spec.json').write_text(json.dumps(zero))
            with self.assertRaisesRegex(ValueError, 'invalid value or bounds for bad_ratio'):
                load_lazy(root)

    def test_source_changed_during_evaluation_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_cad(root / 'cad/part.solidcraft', {'units':'mm','features':[{'type':'box','length':160}]})
            spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{
                'parameters':{'length_m':{'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}}}}}
            (root / 'datum').mkdir(parents=True)
            (root / 'datum/spec.json').write_text(json.dumps(spec))
            from datum_interface import lazy as lazy_module
            original = lazy_module._cad_value
            def racing_cad_value(project, mapping, hashes):
                value = original(project, mapping, hashes)
                (root / 'cad/part.solidcraft').write_bytes((root / 'cad/part.solidcraft').read_bytes() + b'changed')
                return value
            lazy_module._cad_value = racing_cad_value
            try:
                with self.assertRaisesRegex(ValueError, 'source changed during evaluation'):
                    load_lazy(root)
            finally:
                lazy_module._cad_value = original

    def test_same_source_changed_between_mappings_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cad = root / 'cad/part.solidcraft'
            write_cad(cad, {'units':'mm','features':[{'type':'box','length':160}]})
            mapping = {'path':'cad/part.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}
            from datum_interface import lazy as lazy_module
            hashes = {}
            lazy_module._cad_value(root, mapping, hashes)
            write_cad(cad, {'units':'mm','features':[{'type':'box','length':200}]})
            with self.assertRaisesRegex(ValueError, 'source changed during evaluation'):
                lazy_module._cad_value(root, mapping, hashes)


if __name__ == '__main__':
    unittest.main()
