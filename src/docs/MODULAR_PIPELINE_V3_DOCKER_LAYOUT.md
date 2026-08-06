# V3 Docker directory reorganisation

## Reason

The current Docker workflow bind-mounts only the host `src/` directory to
`/workspace/src`. V2 incorrectly placed its launcher and YAML file at the
project root, outside the mounted directory.

## V3 changes

- `run_modular_pipeline.sh` moved to `src/run_modular_pipeline.sh`.
- `test_modular_pipeline.sh` added under `src/`.
- configuration moved to `src/configs/modular_pipeline.yaml`.
- runtime documentation moved to `src/docs/`.
- `main_modular.py` now resolves its default configuration from its own parent
  directory, so the default path is `/workspace/src/configs/modular_pipeline.yaml`.
- configuration tests use the same mounted-source-relative path.
- ZIP top level contains only `src/`, so it can still be extracted from the
  host project root without changing Docker Compose.

## Container commands

```bash
cd /workspace/src
./test_modular_pipeline.sh
./run_modular_pipeline.sh
```
