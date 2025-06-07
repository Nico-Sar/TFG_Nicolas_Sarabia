import subprocess
import glob
import os
import re
from datetime import datetime

# Paths to properties files
protocolparametersFilePath = "src/main/resources/protocols/magneticSwarmRec/SwarmReconfig.properties"
ardusimParametersFilePath = "src/main/resources/setup/SimulationParam.properties"

# Log file
logFilePath = "magneticSwarmRec_log.txt"

# Parameter values for experiments
speed = 5.0
groundDistance = 50
flyingDistance = 50
numUAVs = 10
frd = 60
dirFactor = 2.5

# Formation pairs to test (ground -> flying)
formations = ["MATRIX", "LINEAR", "CIRCLE"]
formation_pairs = [
    (g, f)
    for g in formations
    for f in formations
    if g != f
]

# Utility functions to modify .properties files

def _set_property(file_path, key, value):
    pattern = re.compile(rf"^{key}\s*=.*$", re.IGNORECASE)
    replaced = False
    with open(file_path, 'r', encoding='latin1') as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        if pattern.match(line.strip()):
            prefix = line.split('=')[0]
            lines[i] = f"{prefix}= {value}\n"
            replaced = True
            break
    if not replaced:
        lines.append(f"{key}= {value}\n")
    with open(file_path, 'w', encoding='latin1') as f:
        f.writelines(lines)


def writeProtocolParameters(ground, air):
    _set_property(protocolparametersFilePath, 'speed', speed)
    _set_property(protocolparametersFilePath, 'groundFormation', ground)
    _set_property(protocolparametersFilePath, 'flyingFormation', air)
    _set_property(protocolparametersFilePath, 'groundDistance', groundDistance)
    _set_property(protocolparametersFilePath, 'flyingDistance', flyingDistance)
    _set_property(protocolparametersFilePath, 'frd', frd)
    _set_property(protocolparametersFilePath, 'dirFactor', dirFactor)


def writeArduSimParameters(numUAVs):
    _set_property(ardusimParametersFilePath, 'numUAVs', numUAVs)


def removeFoldersAfterError():
    directories_to_remove = glob.glob('virtual_uav_temp_*')
    for directory in directories_to_remove:
        subprocess.run(['rm', '-r', directory])


for ground, air in formation_pairs:
    writeProtocolParameters(ground, air)
    writeArduSimParameters(numUAVs)
    # Execute the simulator using the jar located in the same directory as this
    # script. This allows running the script from any path as long as both files
    # stay together.
    jar_path = os.path.join(os.path.dirname(__file__), 'ArduSim.jar')
    cmd = ['java', '-jar', jar_path, 'simulator-cli', ardusimParametersFilePath]
    try:
        print(f"Running {numUAVs} UAVs: {ground} -> {air}")
        subprocess.run(cmd, timeout=3000)
        duration = None
        if os.path.exists('uav_0_log.csv'):
            with open('uav_0_log.csv') as f:
                for line in f:
                    if line.startswith('FormationChangeDuration(ms)'):
                        duration = float(line.split(',')[1].strip())
                        break
        collisions = 0
        collision_files = sorted(glob.glob('Collisions*.txt'), key=os.path.getmtime)
        if collision_files:
            with open(collision_files[-1]) as f:
                pairs = 0
                for line in f:
                    parts = line.split(':',1)
                    if len(parts) > 1:
                        ids = parts[1].strip()[1:-1]
                        if ids:
                            pairs += len(ids.split(','))
                collisions = pairs // 2
            os.remove(collision_files[-1])
        if os.path.exists('uav_0_log.csv'):
            os.remove('uav_0_log.csv')
        with open(logFilePath, 'a') as file:
            file.write(f"{ground}->{air};{duration};{collisions}\n")
    except subprocess.TimeoutExpired:
        removeFoldersAfterError()
        with open(logFilePath, 'a') as file:
            file.write(f"{ground}->{air};timeout;0\n")

print("Simulation Done")
