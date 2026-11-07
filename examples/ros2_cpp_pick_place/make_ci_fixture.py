"""Create test-only SolidCraft-shaped ZIP inputs for CI, then generate a header."""
import json
from pathlib import Path
import subprocess
import sys
import zipfile

repo = Path(__file__).resolve().parents[2]
root = Path(sys.argv[1]).resolve()
(root / 'cad').mkdir(parents=True, exist_ok=False)
(root / 'datum').mkdir()
parameters = {}
for name, length in [('upper_arm', 180), ('forearm', 160), ('gripper_palm', 50), ('finger', 55)]:
    with zipfile.ZipFile(root / 'cad' / f'{name}.solidcraft', 'w') as archive:
        archive.writestr('document.json', json.dumps({'units':'mm','features':[{'type':'box','length':length}]}))
    parameters[name + '_length_m'] = {'path':f'cad/{name}.solidcraft','pointer':'/features/0/length','source_units':'mm','unit':'m'}
spec = {'schema':'datum.robot.parameter-spec/2','units':'m','worktrees':{'main':{'parameters':parameters,'constants':{'shoulder_height_m':{'value':.11,'unit':'m'}}}}}
path = root / 'datum/spec.json'
path.write_text(json.dumps(spec))
subprocess.run(['cargo','run','--quiet','--locked','--manifest-path',str(repo/'rust/Cargo.toml'),'--bin','datum-params','--','generate',str(root),str(path),'main',str(root/'datum/parameters.json'),str(root/'datum/parameters.hpp')],check=True)
