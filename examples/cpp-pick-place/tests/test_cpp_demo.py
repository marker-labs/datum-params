"""Exercise generated C++ headers and lazy C++ evaluation without SolidCraft."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / 'examples/cpp-pick-place'

def write_cad_project(root, variant, forearm_length):
    (root / 'cad').mkdir(parents=True)
    (root / 'datum').mkdir()
    parameters = {}
    for name, value in [('upper_arm', 180), ('forearm', forearm_length), ('gripper_palm', 50), ('finger', 55)]:
        with zipfile.ZipFile(root / f'cad/{name}.solidcraft', 'w') as z:
            z.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':value}]}))
        parameters[name + '_length_m'] = {'path':f'cad/{name}.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}
    spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{'parameters':parameters,'constants':{'shoulder_height_m':{'value':.11,'unit':'m'}}}}}
    spec_path = root / 'datum/spec.json'
    spec_path.write_text(json.dumps(spec))
    return spec_path

class CppPickPlaceTest(unittest.TestCase):
    def test_startup_benchmark_compares_matching_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = Path(tmp) / 'changed'
            write_cad_project(project, 'changed', 160)
            subprocess.run(['cargo','build','--quiet','--locked','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--lib'],check=True)
            report = Path(tmp) / 'report.json'
            subprocess.run(['python3',str(EXAMPLE/'benchmark.py'),str(project),'--runs','2','--warmup','0','--output',str(report)],check=True,capture_output=True,text=True)
            data = json.loads(report.read_text())
            self.assertEqual(data['schema'], 'datum.cpp-pick-place.startup-benchmark/1')
            self.assertEqual(len(data['end_to_end_process_ms']['compiled']['samples_ms']), 2)
            self.assertEqual(len(data['end_to_end_process_ms']['lazy']['samples_ms']), 2)
            self.assertGreater(data['one_time_ms']['generate_artifacts'], 0)
            self.assertEqual(len(data['sources_sha256']), 4)

    def test_generated_headers_drive_same_changed_plant(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for variant, length in [('baseline', 100), ('changed', 160)]:
                project = root / variant
                spec_path = write_cad_project(project, variant, length)
                subprocess.run(['cargo','run','--quiet','--locked','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(project),str(spec_path),'main',str(project/'datum/parameters.json'),str(project/'datum/parameters.hpp')],check=True)
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'changed/datum'),'-c',str(EXAMPLE/'physical.cpp'),'-o',str(root/'physical.o')],check=True)
            for variant, expected in [('baseline','fail'),('changed','pass')]:
                executable=root/variant/'pick_place'
                subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/variant/'datum'),str(EXAMPLE/'controller.cpp'),str(root/'physical.o'),'-o',str(executable)],check=True)
                result=subprocess.run([str(executable),expected],check=True,capture_output=True,text=True).stdout
                self.assertIn('physical tool=0.2375 m',result)
                if variant=='baseline':
                    self.assertIn('error=0.0600 m',result)
                    self.assertIn('remaining stages skipped',result)
                else:
                    for stage in ('grasp:', 'lift:', 'place:', 'PASS:'):
                        self.assertIn(stage,result)
                wrong='pass' if expected=='fail' else 'fail'
                self.assertNotEqual(subprocess.run([str(executable),wrong],capture_output=True).returncode,0)

    def test_lazy_evaluation_matches_compiled_values_without_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for variant, length in [('baseline', 100), ('changed', 160)]:
                project = root / variant
                spec_path = write_cad_project(project, variant, length)
                subprocess.run(['cargo','run','--quiet','--locked','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(project),str(spec_path),'main',str(project/'datum/parameters.json'),str(project/'datum/parameters.hpp')],check=True)
            subprocess.run(['cargo','build','--quiet','--locked','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--lib'],check=True)
            executable=root/'pick_place_lazy'
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'rust/include'),'-I',str(EXAMPLE),str(EXAMPLE/'controller_lazy.cpp'),'-o',str(executable),'-L',str(ROOT/'rust/target/debug'),'-ldatum_params_tools','-lpthread','-ldl'],check=True)
            env = dict(os.environ)
            existing = env.get('LD_LIBRARY_PATH')
            env['LD_LIBRARY_PATH'] = str(ROOT/'rust/target/debug') + (os.pathsep + existing if existing else '')
            # The lazy consumer needs no generated artifacts: remove all of them.
            for variant in ('baseline', 'changed'):
                for name in ('parameters.json', 'parameters.hpp'):
                    (root/variant/'datum'/name).unlink()
            for projects, expected in [((root/'changed',root/'changed'),'pass'), ((root/'baseline',root/'changed'),'fail')]:
                evidence = root/f'{projects[0].name}-lazy-evidence.json'
                result=subprocess.run([str(executable),str(projects[0]),str(projects[1]),expected,str(evidence)],check=True,capture_output=True,text=True,env=env).stdout
                self.assertIn(f'controller tool={0.1775 if expected=="fail" else 0.2375} m',result)
                self.assertIn('physical tool=0.2375 m',result)
                if expected=='fail':
                    self.assertIn('error=0.0600 m',result)
                else:
                    self.assertIn('PASS:',result)
                record=json.loads(evidence.read_text())
                self.assertEqual(record['evidence_schema'],'datum.robot.lazy-evidence/1')
                self.assertEqual(record['mode'],'lazy')
                self.assertEqual(record['sources_sha256']['cad/forearm.solidcraft'],hashlib.sha256((projects[0]/'cad/forearm.solidcraft').read_bytes()).hexdigest())
                self.assertEqual(record['spec_sha256'],hashlib.sha256((projects[0]/'datum/spec.json').read_bytes()).hexdigest())
                # The physical evaluation is recorded beside the controller one.
                physical=json.loads(Path(str(evidence)+'.physical.json').read_text())
                self.assertEqual(physical['sources_sha256']['cad/forearm.solidcraft'],hashlib.sha256((projects[1]/'cad/forearm.solidcraft').read_bytes()).hexdigest())
            # Mismatching CAD directories must return the failing exit code.
            self.assertEqual(subprocess.run([str(executable),str(root/'baseline'),str(root/'changed'),'pass'],capture_output=True,text=True,env=env).returncode,1)
            # A failed read still writes the evidence record for the CAD it consumed.
            broken=json.loads((root/'changed/datum/spec.json').read_text())
            del broken['worktrees']['main']['parameters']['finger_length_m']
            (root/'changed/datum/spec.json').write_text(json.dumps(broken))
            failure_evidence=root/'failure-evidence.json'
            result=subprocess.run([str(executable),str(root/'changed'),str(root/'changed'),'pass',str(failure_evidence)],capture_output=True,text=True,env=env)
            self.assertEqual(result.returncode,2)
            self.assertIn('unknown lazy parameter finger_length_m',result.stderr)
            self.assertEqual(json.loads(failure_evidence.read_text())['mode'],'lazy')

    def test_c_abi_buffer_and_session_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project = root / 'project'
            spec_path = write_cad_project(project, 'changed', 160)
            subprocess.run(['cargo','build','--quiet','--locked','--manifest-path',str(ROOT/'rust/Cargo.toml'),'--lib'],check=True)
            source = root / 'contract.cpp'
            source.write_text(r"""
#include "dparams_lazy.h"
#include <cstring>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 4) return 1;
    // A failed evaluation records an error and leaves no session behind.
    if (dparams_lazy_eval(argv[1], argv[3], "main") == 0) return 9;
    if (dparams_lazy_count() != -1) return 10;
    // A null buffer with length 0 queries the required size; a short buffer
    // reports that size and writes nothing.
    const int needed = dparams_lazy_error(nullptr, 0);
    if (needed <= 4) return 11;
    char tiny[4];
    std::memset(tiny, 'x', sizeof tiny);
    if (dparams_lazy_error(tiny, sizeof tiny) != needed) return 12;
    if (tiny[0] != 'x') return 13;
    if (dparams_lazy_eval(argv[1], argv[2], "main") != 0) return 14;
    if (dparams_lazy_count() <= 0) return 15;
    // A successful session call clears the recorded error.
    if (dparams_lazy_error(nullptr, 0) != 1) return 16;
    // A too-small hash buffer reports the size and writes nothing.
    const int hash_needed = dparams_lazy_source_hash("cad/forearm.solidcraft", nullptr, 0);
    if (hash_needed <= 0) return 14;
    char partial[8];
    std::memset(partial, 'x', sizeof partial);
    if (dparams_lazy_source_hash("cad/forearm.solidcraft", partial, sizeof partial) != hash_needed) return 15;
    if (partial[0] != 'x') return 16;
    char full[65];
    if (dparams_lazy_source_hash("cad/forearm.solidcraft", full, sizeof full) != 0) return 17;
    if (std::strlen(full) != 64) return 18;
    // A later failed evaluation clears the session instead of serving old values.
    if (dparams_lazy_eval(argv[1], argv[3], "main") == 0) return 19;
    if (dparams_lazy_count() != -1) return 20;
    if (dparams_lazy_error(nullptr, 0) <= 1) return 21;
    dparams_lazy_release();
    return 0;
}
""")
            executable = root / 'contract'
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'rust/include'),str(source),'-o',str(executable),'-L',str(ROOT/'rust/target/debug'),'-ldatum_params_tools','-lpthread','-ldl'],check=True)
            env = dict(os.environ)
            env['LD_LIBRARY_PATH'] = str(ROOT/'rust/target/debug') + (os.pathsep + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
            result = subprocess.run([str(executable),str(project),str(spec_path),str(root/'missing.json')],capture_output=True,text=True,env=env)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_unreachable_target_is_reported_on_stderr(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'reach.cpp'
            source.write_text(r"""
#include "plan.hpp"
#include <sstream>
#include <string>

int main() {
    const ArmPlan plan{0.01, 0.0, 0.01};
    std::ostringstream out, err;
    const int code = run_plan(plan, 0.01, 0.015, out, err);
    if (code != 2) return 1;
    if (out.str().find("outside controller reach") != std::string::npos) return 2;
    if (err.str().find("outside controller reach") == std::string::npos) return 3;
    return 0;
}
""")
            executable = root / 'reach'
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(EXAMPLE),str(source),'-o',str(executable)],check=True)
            self.assertEqual(subprocess.run([str(executable)]).returncode,0)

if __name__=='__main__': unittest.main()
