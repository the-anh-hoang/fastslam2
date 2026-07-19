#!/usr/bin/env python3

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# --- palette (light mode) ---
SERIES = {  # fixed slot order; extras take the remaining slots
    "fastslam": "#2a78d6",   # blue
    "gmapping": "#008300",   # green
    "odom":     "#e87ba4",   # magenta
}
EXTRA_SLOTS = ["#eda100", "#1baf7a", "#eb6834", "#4a3aa7", "#e34948"]
MISMATCH_RED = "#e34948"
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
# sequential blue ramp, light -> dark (steps 100..700)
SEQ_RAMP = LinearSegmentedColormap.from_list("seq_blue", [
    "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b",
])

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "text.color": INK,
    "axes.labelcolor": INK_SECONDARY,
    "axes.edgecolor": AXIS,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "axes.grid": True,
    "axes.axisbelow": True,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "font.family": "sans-serif",
    "legend.frameon": False,
})


def normalize_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


class Trajectory:
    """A planar TUM trajectory with pose interpolation."""

    def __init__(self, path):
        self.name = Path(path).stem
        data = np.loadtxt(path)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        self.t = data[:, 0]
        self.x = data[:, 1]
        self.y = data[:, 2]
        qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.yaw_unwrapped = np.unwrap(yaw)  # continuous, so np.interp is shortest-arc

        order = np.argsort(self.t)
        if not np.all(order == np.arange(len(self.t))):
            self.t, self.x, self.y = self.t[order], self.x[order], self.y[order]
            self.yaw_unwrapped = self.yaw_unwrapped[order]

    def sample(self, times, max_dt):
        """Interpolate poses at `times`. Returns (x, y, yaw, valid)."""
        times = np.asarray(times)
        inside = (times >= self.t[0]) & (times <= self.t[-1])
        # span between the bracketing samples must be small enough to trust
        # linear interpolation
        right = np.clip(np.searchsorted(self.t, times), 1, len(self.t) - 1)
        span = self.t[right] - self.t[right - 1]
        valid = inside & (span <= max_dt)
        x = np.interp(times, self.t, self.x)
        y = np.interp(times, self.t, self.y)
        yaw = normalize_angle(np.interp(times, self.t, self.yaw_unwrapped))
        return x, y, yaw, valid


def load_relations(path):
    """Freiburg relations: t1 t2 dx dy dz droll dpitch dyaw (planar: dx dy dyaw)."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return {
        "t1": data[:, 0], "t2": data[:, 1],
        "dx": data[:, 2], "dy": data[:, 3], "dyaw": data[:, 7],
    }


def evaluate(traj, rel, max_dt):
    """Per-relation errors of `traj` against ground-truth relations."""
    x1, y1, yaw1, v1 = traj.sample(rel["t1"], max_dt)
    x2, y2, yaw2, v2 = traj.sample(rel["t2"], max_dt)
    valid = v1 & v2

    # estimated relative transform, expressed in pose1's frame
    dx_w, dy_w = x2 - x1, y2 - y1
    c, s = np.cos(-yaw1), np.sin(-yaw1)
    dx_est = c * dx_w - s * dy_w
    dy_est = s * dx_w + c * dy_w
    dyaw_est = normalize_angle(yaw2 - yaw1)

    e_trans = np.hypot(dx_est - rel["dx"], dy_est - rel["dy"])
    e_rot = np.abs(normalize_angle(dyaw_est - rel["dyaw"]))

    return {
        "valid": valid,
        "t1": rel["t1"],
        "e_trans": e_trans,
        "e_rot_deg": np.degrees(e_rot),
        # pose at t1 and both endpoints at t2, for the mismatch figure
        "p1": np.column_stack([x1, y1]),
        "yaw1": yaw1,
        "p2_est": np.column_stack([x2, y2]),
    }


def stats(errors):
    return {
        "mean": float(np.mean(errors)),
        "std": float(np.std(errors)),
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "max": float(np.max(errors)),
    }


def load_map(yaml_path):
    """Minimal map_saver-format loader: returns (image array, extent)."""
    meta = {}
    yaml_path = Path(yaml_path)
    for line in yaml_path.read_text().splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")]
    img = plt.imread(yaml_path.parent / meta["image"])
    h, w = img.shape[:2]
    extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    return img, extent


def series_color(name, taken=[0]):
    if name in SERIES:
        return SERIES[name]
    color = EXTRA_SLOTS[taken[0] % len(EXTRA_SLOTS)]
    taken[0] += 1
    return color


def style_axes(ax, xlabel, ylabel):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)


# ------------------------- figures -------------------------

def fig_trajectories(results, out_dir):
    fig, ax = plt.subplots(figsize=(8, 8))
    for name, r in results.items():
        traj = r["traj"]
        ax.plot(traj.x, traj.y, color=r["color"], linewidth=1.4, label=name)
        ax.plot(traj.x[0], traj.y[0], "o", color=r["color"], markersize=7,
                markerfacecolor=SURFACE, markeredgewidth=1.6)
        ax.plot(traj.x[-1], traj.y[-1], "s", color=r["color"], markersize=7)
        ax.annotate(name, (traj.x[-1], traj.y[-1]), xytext=(6, 6),
                    textcoords="offset points", fontsize=9, color=INK_SECONDARY)
    ax.set_aspect("equal")
    style_axes(ax, "x (m)", "y (m)")
    ax.set_title("Trajectories (o start, ■ end)", color=INK)
    ax.legend(loc="best", fontsize=9)
    fig.savefig(out_dir / "eval_trajectories.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_map_overlay(results, map_data, out_dir):
    img, extent = map_data
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img, cmap="gray", extent=extent, origin="upper",
              vmin=0, vmax=255 if img.dtype == np.uint8 else 1)
    name = "fastslam" if "fastslam" in results else next(iter(results))
    traj = results[name]["traj"]
    ax.plot(traj.x, traj.y, color=results[name]["color"], linewidth=1.4, label=name)
    ax.plot(traj.x[0], traj.y[0], "o", color=results[name]["color"], markersize=7,
            markerfacecolor=SURFACE, markeredgewidth=1.6)
    ax.plot(traj.x[-1], traj.y[-1], "s", color=results[name]["color"], markersize=7)
    ax.grid(False)
    ax.set_aspect("equal")
    style_axes(ax, "x (m)", "y (m)")
    ax.set_title(f"{name} trajectory on map", color=INK)
    ax.legend(loc="best", fontsize=9)
    fig.savefig(out_dir / "eval_map_overlay.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_error_over_time(results, out_dir):
    fig, (ax_t, ax_r) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for name, r in results.items():
        ev = r["eval"]
        t = ev["t1"][ev["valid"]] - ev["t1"].min()
        order = np.argsort(t)
        ax_t.plot(t[order], ev["e_trans"][ev["valid"]][order], ".",
                  color=r["color"], markersize=3, alpha=0.6, label=name)
        ax_r.plot(t[order], ev["e_rot_deg"][ev["valid"]][order], ".",
                  color=r["color"], markersize=3, alpha=0.6, label=name)
    ax_t.set_ylabel("translational error (m)")
    ax_r.set_ylabel("rotational error (deg)")
    ax_r.set_xlabel("dataset time (s)")
    ax_t.set_title("Per-relation error over the run", color=INK)
    ax_t.legend(loc="upper left", fontsize=9, markerscale=3)
    fig.savefig(out_dir / "eval_error_over_time.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_error_hist(results, out_dir):
    fig, (ax_t, ax_r) = plt.subplots(1, 2, figsize=(11, 4))
    all_trans = np.concatenate([r["eval"]["e_trans"][r["eval"]["valid"]] for r in results.values()])
    all_rot = np.concatenate([r["eval"]["e_rot_deg"][r["eval"]["valid"]] for r in results.values()])
    bins_t = np.linspace(0, np.percentile(all_trans, 99), 40)
    bins_r = np.linspace(0, np.percentile(all_rot, 99), 40)
    for name, r in results.items():
        ev = r["eval"]
        ax_t.hist(ev["e_trans"][ev["valid"]], bins=bins_t, histtype="step",
                  linewidth=1.6, color=r["color"], label=name)
        ax_r.hist(ev["e_rot_deg"][ev["valid"]], bins=bins_r, histtype="step",
                  linewidth=1.6, color=r["color"], label=name)
    ax_t.set_xlabel("translational error (m)")
    ax_r.set_xlabel("rotational error (deg)")
    ax_t.set_ylabel("relations")
    ax_t.legend(fontsize=9)
    fig.suptitle("Error distributions (clipped at p99)", color=INK)
    fig.savefig(out_dir / "eval_error_hist.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_mismatch(results, out_dir, n_worst=15):
    name = "fastslam" if "fastslam" in results else next(iter(results))
    r = results[name]
    traj, ev = r["traj"], r["eval"]
    valid = ev["valid"]

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(traj.x, traj.y, color=INK, linewidth=1.0, zorder=1)

    # path points at each relation's t1, colored by local translational error
    vmax = np.percentile(ev["e_trans"][valid], 95)
    # sc = ax.scatter(ev["p1"][valid, 0], ev["p1"][valid, 1],
    #                 c=np.clip(ev["e_trans"][valid], 0, vmax),
    #                 cmap=SEQ_RAMP, s=14, zorder=2)
    # fig.colorbar(sc, ax=ax, shrink=0.7, label="translational relation error (m), clipped at p95")

    # worst relations: estimated motion vs ground-truth motion from the same pose
    idx = np.where(valid)[0]
    worst = idx[np.argsort(ev["e_trans"][idx])[-n_worst:]]
    rel_dx, rel_dy = r["rel"]["dx"], r["rel"]["dy"]
    for k in worst:
        p1 = ev["p1"][k]
        p2_est = ev["p2_est"][k]
        c, s = math.cos(ev["yaw1"][k]), math.sin(ev["yaw1"][k])
        p2_gt = p1 + np.array([c * rel_dx[k] - s * rel_dy[k],
                               s * rel_dx[k] + c * rel_dy[k]])
        ax.plot([p1[0], p2_est[0]], [p1[1], p2_est[1]],
                color=INK_SECONDARY, linewidth=1.2, zorder=3)
        ax.plot([p1[0], p2_gt[0]], [p1[1], p2_gt[1]],
                color=INK_SECONDARY, linewidth=1.2, linestyle=":", zorder=3)
        ax.plot([p2_est[0], p2_gt[0]], [p2_est[1], p2_gt[1]],
                color=MISMATCH_RED, linewidth=1.6, zorder=4)
    ax.plot([], [], color=INK_SECONDARY, linewidth=1.2, label="estimated motion")
    ax.plot([], [], color=INK_SECONDARY, linewidth=1.2, linestyle=":", label="ground-truth motion")
    ax.plot([], [], color=MISMATCH_RED, linewidth=1.6, label=f"mismatch ({n_worst} worst)")

    ax.set_aspect("equal")
    style_axes(ax, "x (m)", "y (m)")
    ax.set_title(f"{name}: where the estimate mismatches ground truth", color=INK)
    ax.legend(loc="best", fontsize=9)
    fig.savefig(out_dir / "eval_mismatch.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


# ------------------------- main -------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results_dir", help="directory containing *.tum trajectories")
    ap.add_argument("relations", help="Freiburg-benchmark .relations ground truth")
    ap.add_argument("--map", help="map.yaml for the map-overlay figure")
    ap.add_argument("--out", help="output directory (default: results_dir)")
    ap.add_argument("--max-dt", type=float, default=2.0,
                    help="max span (s) between trajectory samples bracketing a "
                         "relation timestamp (default: 2.0)")
    args = ap.parse_args()

    results_dir = Path(args.results_dir)
    out_dir = Path(args.out) if args.out else results_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    tum_files = sorted(results_dir.glob("*.tum"))
    if not tum_files:
        sys.exit(f"No .tum files found in {results_dir}")
    rel = load_relations(args.relations)
    n_rel = len(rel["t1"])
    print(f"{n_rel} ground-truth relations from {args.relations}\n")

    results = {}
    for f in tum_files:
        traj = Trajectory(f)
        ev = evaluate(traj, rel, args.max_dt)
        results[traj.name] = {
            "traj": traj, "eval": ev, "rel": rel,
            "color": series_color(traj.name),
        }

    # --- metrics table + json ---
    metrics = {}
    header = (f"{'trajectory':<12} {'trans mean±std':>18} {'rmse':>8} {'max':>8}"
              f" | {'rot mean±std':>16} {'rmse':>8} {'max':>8} | {'used':>6}/{n_rel}")
    print(header)
    print("-" * len(header))
    for name, r in results.items():
        ev = r["eval"]
        used = int(np.count_nonzero(ev["valid"]))
        if used == 0:
            print(f"{name:<12} no relations matched (check timestamps / --max-dt)")
            continue
        st = stats(ev["e_trans"][ev["valid"]])
        sr = stats(ev["e_rot_deg"][ev["valid"]])
        metrics[name] = {"translational_m": st, "rotational_deg": sr,
                         "relations_used": used, "relations_total": n_rel}
        print(f"{name:<12} {st['mean']:>8.3f} ± {st['std']:<6.3f} {st['rmse']:>8.3f} {st['max']:>8.3f}"
              f" | {sr['mean']:>6.2f} ± {sr['std']:<6.2f} {sr['rmse']:>8.2f} {sr['max']:>8.2f}"
              f" | {used:>6}/{n_rel}")

    with open(out_dir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)

    # --- figures ---
    results = {k: v for k, v in results.items() if np.any(v["eval"]["valid"])}
    if results:
        fig_trajectories(results, out_dir)
        fig_error_over_time(results, out_dir)
        fig_error_hist(results, out_dir)
        fig_mismatch(results, out_dir)
        if args.map:
            fig_map_overlay(results, load_map(args.map), out_dir)

    written = ["metrics.json", "eval_trajectories.png", "eval_error_over_time.png",
               "eval_error_hist.png", "eval_mismatch.png"]
    if args.map:
        written.append("eval_map_overlay.png")
    print(f"\nWrote {', '.join(written)} to {out_dir}")


if __name__ == "__main__":
    main()
