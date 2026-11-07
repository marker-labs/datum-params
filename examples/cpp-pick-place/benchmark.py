#!/usr/bin/env python3
"""Measure this kinematic example's generated and lazy startup paths."""
import argparse
import hashlib
import json
import platform
import statistics
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / 'examples/cpp-pick-place'


def run(command):
    start = time.perf_counter_ns()
    result = subprocess.run([str(x) for x in command], check=True, capture_output=True, text=True)
    return (time.perf_counter_ns() - start) / 1e6, result.stdout


def summary(samples):
    return {'samples_ms': samples, 'median_ms': statistics.median(samples),
            'min_ms': min(samples), 'max_ms': max(samples)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', type=Path, help='changed project produced by run.sh')
    parser.add_argument('--runs', type=int, default=20)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--output', type=Path, help='optional JSON results file')
    args = parser.parse_args()
    if args.runs < 1 or args.warmup < 0:
        parser.error('--runs must be positive and --warmup non-negative')
    project = args.project.resolve(strict=True)
    spec = project / 'datum/spec.json'
    generator = ROOT / 'rust/target/debug/datum-params'
    library = ROOT / 'rust/target/debug/libdatum_params_tools.a'
    if not spec.is_file() or not generator.is_file() or not library.is_file():
        parser.error('project spec or Rust debug build missing; run run.sh first')
    with tempfile.TemporaryDirectory(prefix='dparams-bench-') as temp:
        out = Path(temp)
        generation_ms, _ = run([generator, 'generate', project, spec, 'main', out/'parameters.json', out/'parameters.hpp'])
        manifest = json.loads((out/'parameters.json').read_text())
        build_ms, _ = run(['c++', '-std=c++17', '-O2', '-I', out, EXAMPLE/'controller.cpp', EXAMPLE/'physical.cpp', '-o', out/'compiled'])
        lazy_build_ms, _ = run(['c++', '-std=c++17', '-O2', '-I', ROOT/'rust/include', '-I', EXAMPLE, EXAMPLE/'controller_lazy.cpp', library, '-lpthread', '-ldl', '-o', out/'lazy'])
        compiled = [out/'compiled', 'pass']
        lazy = [out/'lazy', project, project, 'pass']
        # Verify the exact same plan and result before reporting timings.
        _, compiled_result = run(compiled)
        _, lazy_result = run(lazy)
        if compiled_result != lazy_result:
            raise RuntimeError('compiled and lazy plan output differs; refusing to compare')
        samples = {'compiled': [], 'lazy': []}
        for iteration in range(args.warmup + args.runs):
            # Alternate order to reduce a systematic ordering bias.
            modes = [('compiled', compiled), ('lazy', lazy)]
            if iteration % 2:
                modes.reverse()
            for name, command in modes:
                elapsed, output = run(command)
                if output != compiled_result:
                    raise RuntimeError(f'{name} output changed during benchmark')
                if iteration >= args.warmup:
                    samples[name].append(elapsed)
        for name, digest in manifest['sources_sha256'].items():
            if hashlib.sha256((project/name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f'CAD source changed during benchmark: {name}')
        if hashlib.sha256(spec.read_bytes()).hexdigest() != manifest['spec_sha256']:
            raise RuntimeError('spec changed during benchmark')
        report = {'schema': 'datum.cpp-pick-place.startup-benchmark/1',
                  'platform': platform.platform(), 'project': str(project),
                  'spec_sha256': manifest['spec_sha256'], 'sources_sha256': manifest['sources_sha256'],
                  'runs': args.runs, 'warmup': args.warmup,
                  'one_time_ms': {'generate_artifacts': generation_ms, 'compile_generated_example': build_ms,
                                  'compile_lazy_example': lazy_build_ms},
                  'end_to_end_process_ms': {k: summary(v) for k, v in samples.items()},
                  'scope': 'Kinematic example process start through completion; lazy evaluates CAD twice, compiled uses baked constants. No URDF, ROS, physics, or runtime freshness check.'}
        text = json.dumps(report, indent=2) + '\n'
        if args.output:
            args.output.write_text(text)
        print(text, end='')


if __name__ == '__main__':
    main()
