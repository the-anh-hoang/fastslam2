#!/usr/bin/env python3
"""Run a SLAM benchmark reproducibly.

Wraps the offline runners (fastslam clf_runner, gmapping_clf_runner) so every
run lands in its own results/ directory containing everything needed to
reproduce and judge it:

  results/<dataset>_<algo>_<tag|timestamp>/
    config.yaml       snapshot of the config actually used
    run.json          command line, seed, git commit, dataset, timestamps
    run.log           full runner output
    *.tum             trajectories
    map.pgm/.yaml     final map
    metrics.json      relation-metric scores (if ground truth exists)
    eval_*.png        evaluation figures

Usage (from the repo root):
  python3 tools/run_benchmark.py intel fastslam --seed 42 --tag tuned
  python3 tools/run_benchmark.py mit-killian gmapping --seed 42
"""

import argparse
import datetime
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNERS = {
    "fastslam": ROOT / "ros2_ws/build/fastslam/clf_runner",
    "gmapping": ROOT / "ros2_ws/build/slam_gmapping/gmapping_clf_runner",
}


def git_state():
    def run(*args):
        try:
            return subprocess.run(["git", *args], cwd=ROOT, capture_output=True,
                                  text=True, check=True).stdout.strip()
        except Exception:
            return None
    sha = run("rev-parse", "HEAD")
    dirty = bool(run("status", "--porcelain")) if sha else None
    return {"commit": sha, "dirty": dirty}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="dataset name (datasets/<name>.clf), e.g. intel")
    ap.add_argument("algo", choices=sorted(RUNNERS), help="which SLAM runner")
    ap.add_argument("--config", default="default",
                    help="config name (configs/<algo>/<name>.yaml) or a path to "
                         "any yaml file, default: default")
    ap.add_argument("--seed", type=int, default=None,
                    help="RNG seed for reproducible runs (default: random)")
    ap.add_argument("--tag", help="run directory suffix (default: timestamp)")
    ap.add_argument("--no-eval", action="store_true",
                    help="skip evaluate_slam.py even if ground truth exists")
    args = ap.parse_args()

    clf = ROOT / "datasets" / f"{args.dataset}.clf"
    # --config is either a name under configs/<algo>/ or a path to a yaml file
    candidate = Path(args.config).expanduser()
    if candidate.suffix in (".yaml", ".yml") or candidate.exists():
        config = candidate.resolve()
    else:
        config = ROOT / "configs" / args.algo / f"{args.config}.yaml"
    relations = ROOT / "datasets" / f"{args.dataset}.relations"
    runner = RUNNERS[args.algo]
    for path, what in [(clf, "dataset"), (config, "config"), (runner, "runner (colcon build first?)")]:
        if not path.exists():
            sys.exit(f"Missing {what}: {path}")

    tag = args.tag or datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = ROOT / "results" / f"{args.dataset}_{args.algo}_{tag}"
    if run_dir.exists():
        sys.exit(f"Run directory already exists: {run_dir}")
    run_dir.mkdir(parents=True)

    config_snapshot = run_dir / "config.yaml"
    shutil.copy(config, config_snapshot)

    cmd = [str(runner), str(clf), "--params", str(config_snapshot), "--out", str(run_dir)]
    if args.seed is not None:
        cmd += ["--seed", str(args.seed)]

    meta = {
        "dataset": str(clf.relative_to(ROOT)),
        "algo": args.algo,
        "config": str(config.relative_to(ROOT)) if config.is_relative_to(ROOT) else str(config),
        "seed": args.seed,
        "command": cmd,
        "git": git_state(),
        "started": datetime.datetime.now().isoformat(timespec="seconds"),
    }

    print(f"Run dir: {run_dir.relative_to(ROOT)}")
    print(f"$ {' '.join(cmd)}\n")
    log_path = run_dir / "run.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:
            sys.stdout.write(line)
            log.write(line)
        proc.wait()

    meta["finished"] = datetime.datetime.now().isoformat(timespec="seconds")
    meta["exit_code"] = proc.returncode
    with open(run_dir / "run.json", "w") as f:
        json.dump(meta, f, indent=2)
    if proc.returncode != 0:
        sys.exit(f"Runner failed (exit {proc.returncode}), see {log_path}")

    if args.no_eval:
        return
    if not relations.exists():
        print(f"No ground truth ({relations.name}) — skipping evaluation")
        return
    eval_cmd = [sys.executable, str(ROOT / "tools/evaluate_slam.py"),
                str(run_dir), str(relations)]
    if (run_dir / "map.yaml").exists():
        eval_cmd += ["--map", str(run_dir / "map.yaml")]
    print(f"\n$ {' '.join(eval_cmd)}\n")
    subprocess.run(eval_cmd, cwd=ROOT, check=False)


if __name__ == "__main__":
    main()
