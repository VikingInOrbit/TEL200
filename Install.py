import os
import subprocess
import sys

# Path to venv and requirements
venv_dir = os.path.join(os.getcwd(), '.venv')
requirements_file = os.path.join(os.getcwd(), 'requirements.in')

# 1. Create venv if it doesn't exist
if not os.path.isdir(venv_dir):
    print('Creating virtual environment...')
    subprocess.check_call([sys.executable, '-m', 'venv', venv_dir])
else:
    print('Virtual environment already exists.')

# 2. Install dependencies
pip_exe = os.path.join(venv_dir, 'Scripts', 'pip.exe') if os.name == 'nt' else os.path.join(venv_dir, 'bin', 'pip')
print('Installing dependencies from requirements.in...')
subprocess.check_call([pip_exe, 'install', '-r', requirements_file])

# 3. Patch problematic files in roboticstoolbox
def patch_file(path):
    if not os.path.exists(path):
        return
    with open(path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    changed = False
    with open(path, 'w', encoding='utf-8') as f:
        for line in lines:
            if 'from numpy import disp' in line:
                f.write('# ' + line)
                changed = True
            else:
                f.write(line)
    if changed:
        print(f'Patched: {path}')

rtb_mobile = os.path.join(venv_dir, 'Lib', 'site-packages', 'roboticstoolbox', 'mobile')
files_to_patch = [
    'DistanceTransformPlanner.py',
    'DstarPlanner-old.py',
    'plot_vehicle.py',
    'reedsshepp-old.py',
]
for fname in files_to_patch:
    patch_file(os.path.join(rtb_mobile, fname))

print('Setup complete!')
