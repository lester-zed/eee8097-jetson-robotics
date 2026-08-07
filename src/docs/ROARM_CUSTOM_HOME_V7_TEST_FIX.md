# RoArm Custom Home V7 test import fix

## Symptom

`test_roarm_home.sh` failed with:

```text
ModuleNotFoundError: No module named 'tests.test_roarm_home'
```

and could emit unrelated Torch/Torchvision warnings.

## Cause

The previous command:

```bash
python3 -m unittest tests.test_roarm_home -v
```

allowed Python to resolve a third-party package named `tests` from the Docker
image instead of `/workspace/src/tests`.

## Fix

- export `/workspace/src` at the front of `PYTHONPATH`;
- run `unittest discover` against the absolute repository test directory;
- add `src/tests/__init__.py` as a local package marker;
- compile the test module before executing it.

## Correct commands

```bash
cd /workspace/src
./test_roarm_home.sh
./run_roarm_home.sh preview
```

`preview` only prints the configured T=122 target and does not open the RoArm
serial port or send a motion command.
