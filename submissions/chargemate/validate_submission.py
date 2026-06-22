from __future__ import annotations

import json
import shutil
from pathlib import Path

from chargemate.benchmark import main as run_benchmark


EXPECTED_UUID = "0d354824-2ac1-41cf-a777-583ce7b93a4e"
EXPECTED_PARTICIPANT = "minernote"
EXPECTED_PROJECT = "ChargeMate: Closed-Loop EV Charging Benchmark"


def _read_registration(root: Path) -> dict:
    registration_path = root / "registration.json"
    if not registration_path.exists():
        raise SystemExit("registration.json is missing")
    return json.loads(registration_path.read_text(encoding="utf-8"))


def _check_registration(registration: dict) -> None:
    expected = {
        "uuid": EXPECTED_UUID,
        "participant_name": EXPECTED_PARTICIPANT,
        "project_name": EXPECTED_PROJECT,
    }
    for key, value in expected.items():
        if registration.get(key) != value:
            raise SystemExit(f"registration.json has invalid {key}: {registration.get(key)!r}")


def _check_outputs(out_dir: Path) -> dict:
    required = [
        out_dir / "metrics.json",
        out_dir / "results.json",
        out_dir / "chargemate_scene.xml",
        out_dir / "chargemate_trace.html",
        out_dir / "episode_000.jsonl",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise SystemExit(f"missing generated outputs: {missing}")

    metrics = json.loads((out_dir / "metrics.json").read_text(encoding="utf-8"))
    if metrics["success_rate"] < 1.0:
        raise SystemExit(f"success_rate below target: {metrics['success_rate']}")
    if metrics["mean_lateral_error_m"] > 0.018:
        raise SystemExit(f"mean_lateral_error_m above target: {metrics['mean_lateral_error_m']}")
    if metrics["mean_depth_error_m"] > 0.026:
        raise SystemExit(f"mean_depth_error_m above target: {metrics['mean_depth_error_m']}")
    return metrics


def main() -> None:
    root = Path(__file__).resolve().parent
    _check_registration(_read_registration(root))

    out_dir = root / "outputs-validation"
    if out_dir.exists():
        shutil.rmtree(out_dir)

    import sys

    old_argv = sys.argv
    try:
        sys.argv = [
            "validate_submission.py",
            "--episodes",
            "3",
            "--seed",
            "42",
            "--randomize",
            "--out",
            str(out_dir),
        ]
        run_benchmark()
    finally:
        sys.argv = old_argv

    metrics = _check_outputs(out_dir)
    print("VALIDATION PASSED")
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
