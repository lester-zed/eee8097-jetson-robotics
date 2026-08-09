from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any

import yaml

from configuration.loader import RuntimeConfig, load_runtime_config
from roarm_home import execute_custom_home, load_home_document


SOURCE_ROOT = Path(__file__).resolve().parent
DEFAULT_PIPELINE_CONFIG = SOURCE_ROOT / "configs/modular_pipeline.yaml"
DEFAULT_HOME_CONFIG = SOURCE_ROOT / "configs/roarm_home.yaml"


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _runtime_config(
    pipeline_path: Path,
    home_document: dict[str, Any],
    pipeline_config: RuntimeConfig | None = None,
) -> tuple[Path, bool]:
    override_section = home_document.get("pipeline_overrides", {})
    if not isinstance(override_section, dict) or not bool(
        override_section.get("enabled", False)
    ):
        return pipeline_path, False
    values = override_section.get("values", {})
    if not isinstance(values, dict):
        raise ValueError("pipeline_overrides.values must be a mapping")
    effective = pipeline_config or load_runtime_config(pipeline_path)
    merged = _deep_merge(effective.data, values)
    handle = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="eee8097_pipeline_runtime_",
        suffix=".yaml",
        delete=False,
    )
    with handle:
        yaml.safe_dump(merged, handle, sort_keys=False, allow_unicode=True)
    return Path(handle.name), True


def should_execute_startup_home(
    pipeline_config: RuntimeConfig,
    home_section: dict[str, Any],
    *,
    skip_home: bool,
) -> bool:
    arm_mode = str(pipeline_config.get("arm", "mode")).lower()
    experiment = pipeline_config.data.get("experiment", {})
    requires_mock_arm = isinstance(experiment, dict) and bool(
        experiment.get("require_mock_arm", False)
    )
    return (
        not skip_home
        and arm_mode == "real"
        and not requires_mock_arm
        and bool(home_section.get("enabled", False))
        and bool(home_section.get("run_before_pipeline", False))
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run custom T=122 home, then launch the modular pipeline"
    )
    parser.add_argument("--config", default=str(DEFAULT_PIPELINE_CONFIG))
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG))
    parser.add_argument(
        "--skip-home",
        action="store_true",
        help="Skip startup home for this run; pipeline speed overrides still apply",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pipeline_path = Path(args.config).expanduser().resolve()
    home_path = Path(args.home_config).expanduser().resolve()
    if not pipeline_path.is_file():
        raise FileNotFoundError(f"Pipeline config not found: {pipeline_path}")
    pipeline_config = load_runtime_config(pipeline_path)
    home_document = load_home_document(home_path)
    home_section = home_document["roarm_home"]

    should_home = should_execute_startup_home(
        pipeline_config,
        home_section,
        skip_home=args.skip_home,
    )
    if (
        not args.skip_home
        and bool(home_section.get("enabled", False))
        and bool(home_section.get("run_before_pipeline", False))
        and not should_home
    ):
        print(
            "Skipping RoArm startup home because the selected pipeline "
            "profile does not permit real arm motion."
        )
    if should_home:
        print("Executing configured T=122 machine-forward home before pipeline...")
        try:
            result = execute_custom_home(home_document)
            print(
                "Custom home complete: "
                f"base={result.final_deg['base']:.1f}°, "
                f"shoulder={result.final_deg['shoulder']:.1f}°, "
                f"elbow={result.final_deg['elbow']:.1f}°"
            )
        except Exception:
            if bool(home_section.get("require_success_before_pipeline", True)):
                raise
            print("WARNING: startup home failed; continuing because require_success=false")

    runtime_path, temporary = _runtime_config(
        pipeline_path,
        home_document,
        pipeline_config,
    )
    if temporary:
        print(f"Runtime pipeline config with YAML speed overrides: {runtime_path}")

    command = [
        sys.executable,
        str(SOURCE_ROOT / "main_modular.py"),
        "--config",
        str(runtime_path),
    ]
    try:
        completed = subprocess.run(command, cwd=str(SOURCE_ROOT), check=False)
        return int(completed.returncode)
    finally:
        if temporary:
            try:
                os.unlink(runtime_path)
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
