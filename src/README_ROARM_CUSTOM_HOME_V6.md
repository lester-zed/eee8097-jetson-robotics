# V6 installation

Extract this archive from the host project root. It contains only `src/` paths
and is compatible with the existing `src:/workspace/src` Docker mount.

```bash
unzip -o eee8097_roarm_custom_home_v6_extract_into_project_root_2026-08-06.zip
chmod +x src/run_modular_pipeline.sh src/run_roarm_home.sh src/test_roarm_home.sh
```

Inside Docker:

```bash
cd /workspace/src
./test_roarm_home.sh
```

Edit:

```text
/workspace/src/configs/roarm_home.yaml
```

Set `allow_real_motion` and `confirm_clearance` to `true`, then test:

```bash
./run_roarm_home.sh home
```

After the custom home is verified, start the pipeline. It will repeat the home
before Camera/LiDAR processing:

```bash
./run_modular_pipeline.sh
```

## V7 test-script correction

The V7 package replaces the module-name test invocation with repository-local
unittest discovery. This avoids a name collision with third-party Docker
packages called `tests`:

```bash
python3 -m unittest discover \
  -s /workspace/src/tests \
  -p 'test_roarm_home.py' \
  -v
```
