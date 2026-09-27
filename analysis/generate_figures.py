#!/usr/bin/env python3
"""Generate RAS manuscript figures and tables from archived inputs.

This script reads existing CSV/JSON, scene YAML/SDF, and the RotorS-native
URDF.  It never starts ROS, Gazebo, RotorS, or any scientific experiment.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.transforms import Bbox
import numpy as np
import pandas as pd
import yaml


PROJECT = Path(__file__).resolve().parents[1]
PAPER = PROJECT
SRC = PROJECT / "ros_ws" / "src"
MISSION = PROJECT / "data" / "stage0"
F1 = PROJECT / "data" / "f1"
D2 = PROJECT / "data" / "d2"
E1R2 = PROJECT / "data" / "rotors_r2"
CVAR = PROJECT / "data" / "cvar"
FIG = PROJECT / "reproduced" / "figures"
TAB = PROJECT / "reproduced" / "tables"
EVID = PROJECT / "reproduced" / "evidence"

COL = {
    "fixed": "#5D6D7E",
    "threshold": "#D89000",
    "risk": "#006BA4",
    "safe": "#2B8A3E",
    "unsafe": "#C43C39",
    "ground": "#4F7C4D",
    "air": "#5B8FC4",
    "hazard": "#E76F51",
    "slope": "#E9C46A",
    "rotors": "#168A83",
    "ink": "#253238",
}


def configure_style() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Liberation Sans", "DejaVu Sans"],
        "font.size": 8.0,
        "axes.titlesize": 8.6,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.0,
        "ytick.labelsize": 7.0,
        "legend.fontsize": 6.8,
        "figure.dpi": 160,
        "savefig.dpi": 400,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.7,
        "lines.linewidth": 1.35,
    })


def save(fig: plt.Figure, stem: str) -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(FIG / f"{stem}.svg", bbox_inches="tight")
    fig.savefig(FIG / f"{stem}.png", bbox_inches="tight", dpi=400)
    fig.savefig(FIG / f"{stem}.tiff", bbox_inches="tight", dpi=600)
    plt.close(fig)


def panel(ax, label: str, x: float = -0.10, y: float = 1.04) -> None:
    ax.text(x, y, label, transform=ax.transAxes, ha="left", va="bottom",
            fontsize=8.5, weight="bold", color=COL["ink"])


def parse_scene(scene: str) -> dict:
    cfg_path = SRC / "rescue_mission" / "config" / "scenes" / f"{scene}.yaml"
    world_path = SRC / "rescue_worlds" / "worlds" / f"disaster_{scene}.world"
    cfg = yaml.safe_load(cfg_path.read_text())
    root = ET.parse(world_path).getroot()
    models = []
    for model in root.findall(".//model"):
        if model.attrib.get("name") == "victim_marker":
            continue
        pose = [float(v) for v in model.findtext("pose").split()]
        size_text = model.findtext(".//collision/geometry/box/size")
        if size_text:
            size = [float(v) for v in size_text.split()]
            models.append({"name": model.attrib["name"], "x": pose[0], "y": pose[1],
                           "yaw": pose[5], "sx": size[0], "sy": size[1]})
    return {"cfg": cfg, "models": models, "cfg_path": cfg_path, "world_path": world_path}


def draw_scene(ax, scene: str) -> None:
    data = parse_scene(scene)
    cfg = data["cfg"]
    sensor = cfg["scenario_sensor_sim"]
    ground = cfg["ground_goal_controller"]
    waypoints = np.asarray(cfg["mission_state_machine"]["waypoints"], dtype=float)[:, :2]

    # Risk fields are configuration semantics; solid obstacles come from SDF.
    ax.axvspan(sensor["slope_zone_x_min"], sensor["slope_zone_x_max"],
               color=COL["slope"], alpha=.25, zorder=0)
    barrier = mpatches.Rectangle(
        (sensor["barrier_x_min"], -sensor["barrier_y_half_width"]),
        sensor["barrier_x_max"] - sensor["barrier_x_min"],
        2 * sensor["barrier_y_half_width"], facecolor=COL["hazard"], alpha=.10,
        edgecolor=COL["hazard"], hatch="////", linewidth=.7, zorder=0,
    )
    ax.add_patch(barrier)
    for obj in data["models"]:
        rectangle = mpatches.Rectangle((-obj["sx"] / 2, -obj["sy"] / 2), obj["sx"], obj["sy"],
                                       facecolor="#777777", edgecolor="#333333", linewidth=.55)
        transform = (mpatches.transforms.Affine2D().rotate(obj["yaw"])
                     .translate(obj["x"], obj["y"]) + ax.transData)
        rectangle.set_transform(transform)
        ax.add_patch(rectangle)
    ax.plot(waypoints[:, 0], waypoints[:, 1], "o-", color=COL["ink"], markersize=2.8,
            linewidth=1.0, zorder=4)
    ax.scatter([0], [0], marker="s", s=28, color=COL["ground"], edgecolor="white",
               linewidth=.5, zorder=5)
    ax.scatter([waypoints[-1, 0]], [waypoints[-1, 1]], marker="*", s=62,
               color=COL["unsafe"], edgecolor="white", linewidth=.5, zorder=5)
    ax.text(.15, -.36, "start", fontsize=6.1)
    ax.text(waypoints[-1, 0] - .55, .32, "goal", fontsize=6.1)
    ax.set_xlim(-.5, max(11.5, waypoints[-1, 0] + .5))
    ax.set_ylim(-2.35, 2.35)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=.13, linewidth=.5)
    ax.set_title(f"Scene {scene[-1].upper()} (recoverable)")


def draw_robot(ax) -> None:
    xacro = (SRC / "uav_v4" / "urdf" / "uav_v4_rotors_native.urdf.xacro").read_text()
    rotor_matches = re.findall(r'<xacro:native_rotor number="(\d+)" xyz="([^"]+)"\s+direction="(\w+)"', xacro)
    rotors = []
    for number, xyz, direction in rotor_matches:
        vals = [float(v) for v in xyz.split()]
        rotors.append((int(number), vals[0], vals[1], direction))
    body = mpatches.FancyBboxPatch((-.075, -.085), .15, .17,
                                   boxstyle="round,pad=.005,rounding_size=.015",
                                   facecolor="#DDE7EA", edgecolor=COL["ink"], linewidth=1.0)
    ax.add_patch(body)
    for number, x, y, direction in rotors:
        ax.plot([0, x], [0, y], color="#666666", linewidth=2.0, zorder=1)
        ax.add_patch(mpatches.Circle((x, y), .065, facecolor="#D7EAF4", alpha=.65,
                                    edgecolor=COL["air"], linewidth=.8, zorder=2))
        ax.scatter([x], [y], s=12, color=COL["ink"], zorder=3)
        ax.text(x, y, str(number), ha="center", va="center", fontsize=5.6,
                color="white", zorder=4)
        ax.annotate("", xy=(x + (.024 if direction == "ccw" else -.024), y + .028),
                    xytext=(x, y + .045), arrowprops=dict(arrowstyle="->", lw=.5,
                    color=COL["air"]))
    for x in (-.04, .04):
        for y in (-.088, .088):
            ax.add_patch(mpatches.Rectangle((x - .020, y - .010), .040, .020,
                                            facecolor="#444", edgecolor="none", zorder=5))
    ax.annotate("forward", xy=(.16, 0), xytext=(.02, 0),
                arrowprops=dict(arrowstyle="->", color=COL["unsafe"], lw=1.0),
                va="center", fontsize=6.2, color=COL["unsafe"])
    ax.set_xlim(-.19, .21)
    ax.set_ylim(-.19, .19)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("RotorS-native uav_v4")
    ax.text(.5, -.03, "top view; 3.0441 kg", transform=ax.transAxes,
            ha="center", va="top", fontsize=6.2)


def system_and_scenes() -> None:
    """Install the author-supplied overview artwork without regenerating it."""
    source = PROJECT / "assets" / "system_overview.png"
    target = FIG / "fig01_system_scenes_evidence.png"
    if not source.exists():
        raise FileNotFoundError(f"Missing author-supplied Figure 1 asset: {source}")
    target.write_bytes(source.read_bytes())

def stage0_regime() -> None:
    rows = []
    for regime in ("permanent", "recoverable"):
        for path in sorted(MISSION.glob(f"summary_regime_scene_b_{regime}_*.csv")):
            row = pd.read_csv(path).iloc[0].to_dict()
            row["regime"] = regime
            if "threshold_t" in path.name:
                row["label"] = "τ=" + path.stem.split("threshold_t")[-1]
                row["family"] = "threshold"
            elif "risk_aware" in path.name:
                row["label"] = "risk-aware"; row["family"] = "risk"
            elif "fixed_switch" in path.name:
                row["label"] = "fixed"; row["family"] = "fixed"
            else:
                row["label"] = "energy rule"; row["family"] = "ground"
            row["source_csv"] = str(path)
            rows.append(row)
    data = pd.DataFrame(rows)
    data.to_csv(TAB / "stage0_scene_b_regime_pilot.csv", index=False)

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.75), sharex=True, sharey=True)
    fig.subplots_adjust(left=.085, right=.985, top=.80, bottom=.22, wspace=.14)

    # Restrained Okabe--Ito colours plus redundant marker shapes retain meaning
    # in grayscale. These local colours do not alter other manuscript figures.
    colours = {
        "threshold": "#0072B2", "fixed": "#4D4D4D",
        "risk": "#D55E00", "ground": "#009E73",
    }
    markers = {"threshold": "o", "fixed": "D", "risk": "^", "ground": "s"}
    titles = {
        "permanent": "Permanent hard-trap model",
        "recoverable": "Recoverable traction-degradation model",
    }

    def place_threshold_labels(ax: plt.Axes, threshold_rows: pd.DataFrame,
                               all_rows: pd.DataFrame) -> None:
        """Greedily choose collision-free, inward-facing label offsets."""
        fig.canvas.draw()
        renderer = fig.canvas.get_renderer()
        point_boxes = []
        for _, point in all_rows.iterrows():
            px, py = ax.transData.transform((point.total_energy_j_mean,
                                             point.success_rate))
            point_boxes.append(Bbox.from_extents(px - 5.5, py - 5.5,
                                                  px + 5.5, py + 5.5))
        occupied = []
        for _, row in threshold_rows.sort_values(
                ["success_rate", "total_energy_j_mean"]).iterrows():
            y = float(row.success_rate)
            if y >= .85:
                candidates = [(7, -7), (-7, -7), (13, -7), (-13, -7),
                              (8, -17), (-8, -17), (18, -17), (-18, -17)]
            elif y <= .15:
                candidates = [(7, 7), (-7, 7), (13, 7), (-13, 7),
                              (8, 17), (-8, 17), (18, 17), (-18, 17)]
            else:
                candidates = [(7, 7), (-7, 7), (7, -7), (-7, -7),
                              (14, 0), (-14, 0), (8, 17), (-8, -17)]

            selected = None
            for dx, dy in candidates:
                ann = ax.annotate(
                    row.label.replace("τ=", ""),
                    (row.total_energy_j_mean, row.success_rate),
                    xytext=(dx, dy), textcoords="offset points",
                    ha="left" if dx >= 0 else "right",
                    va="bottom" if dy >= 0 else "top",
                    fontsize=6.5, color=colours["threshold"], zorder=5,
                )
                fig.canvas.draw()
                box = ann.get_window_extent(renderer=renderer).expanded(1.10, 1.18)
                inside = (box.x0 >= ax.bbox.x0 and box.x1 <= ax.bbox.x1 and
                          box.y0 >= ax.bbox.y0 and box.y1 <= ax.bbox.y1)
                collision = any(box.overlaps(other) for other in occupied)
                collision |= any(box.overlaps(point) for point in point_boxes)
                if inside and not collision:
                    selected = (ann, box)
                    break
                ann.remove()
            if selected is None:
                ann = ax.annotate(
                    row.label.replace("τ=", ""),
                    (row.total_energy_j_mean, row.success_rate),
                    xytext=(7, 7), textcoords="offset points",
                    fontsize=6.5, color=colours["threshold"], zorder=5,
                )
                fig.canvas.draw()
                selected = (ann, ann.get_window_extent(renderer=renderer))
            occupied.append(selected[1])

    for ax, regime, letter in zip(axes, ("permanent", "recoverable"), ("a", "b")):
        sub = data[data.regime == regime]
        for _, row in sub.iterrows():
            ax.scatter(
                row.total_energy_j_mean, row.success_rate, s=50,
                marker=markers[row.family], facecolor=colours[row.family],
                edgecolor="white", linewidth=.7, zorder=3, clip_on=False,
            )
        ax.set_title(titles[regime], pad=7)
        ax.set_xlim(0, 112)
        ax.set_ylim(0, 1)
        ax.set_xticks(np.arange(0, 113, 20))
        ax.set_yticks(np.linspace(0, 1, 5))
        ax.grid(True, color="#D8D8D8", linewidth=.5, alpha=.65)
        ax.set_axisbelow(True)
        ax.text(-.02, 1.06, f"({letter})", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=8.5, weight="bold",
                color=COL["ink"])
        place_threshold_labels(ax, sub[sub.family == "threshold"], sub)

    axes[0].set_ylabel("Mission completion rate")
    fig.supxlabel("Legacy task-effort proxy (unconditional mean units)",
                  y=.055, fontsize=8.0)
    legend_handles = [
        plt.Line2D([], [], linestyle="none", marker=markers[family], markersize=5.8,
                   markerfacecolor=colours[family], markeredgecolor="white",
                   markeredgewidth=.7, label=label)
        for family, label in (("fixed", "Fixed switch"),
                              ("risk", "Risk-aware"),
                              ("ground", "Energy rule"))
    ]
    fig.legend(handles=legend_handles, loc="upper center", ncol=3,
               frameon=False, bbox_to_anchor=(.5, .995),
               handletextpad=.45, columnspacing=1.5)
    save(fig, "fig02_stage0_recoverability_regime")

def f1_heldout() -> None:
    runs = pd.read_csv(F1 / "heldout_run_summary.csv")
    stats = json.loads((F1 / "heldout_paired_statistics.json").read_text())
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 4.35),
                             gridspec_kw={"height_ratios": [1.10, 1.0]},
                             sharex="row")
    methods = [("threshold_switch", "Threshold", COL["threshold"]),
               ("risk_aware", "Risk-aware", COL["risk"]),
               ("fixed_reference", "Fixed ref.", COL["fixed"])]
    for j, scene in enumerate(("scene_b", "scene_c")):
        sub = runs[runs.scene == scene]
        ax = axes[0, j]
        for i, (family, label, color) in enumerate(methods):
            vals = sub[sub.family == family].sort_values("seed")
            ok = vals.success.astype(bool).to_numpy()
            seeds = vals.seed.to_numpy()
            ax.scatter(seeds[ok], np.full(ok.sum(), i), color=color, marker="o", s=22,
                       edgecolor="white", linewidth=.4)
            ax.scatter(seeds[~ok], np.full((~ok).sum(), i), color=COL["unsafe"],
                       marker="x", s=28, linewidth=1.1)
            ax.text(219.8, i, f"{ok.sum()}/20", va="center", fontsize=6.4)
        ax.set_yticks(range(3), [m[1] for m in methods])
        ax.set_xlim(199.2, 221.2); ax.set_ylim(2.55, -.55)
        ax.set_xticks([200, 204, 208, 212, 216, 219])
        ax.set_xlabel("Paired held-out seed")
        p = stats["scenes"][scene]["mcnemar"]["exact_two_sided_p"]
        ax.set_title(f"Scene {scene[-1].upper()}: completion (exact McNemar $p$={p:g})")
        ax.grid(axis="x", alpha=.14)
        panel(ax, "a" if j == 0 else "b", x=.01, y=.94)

        ax = axes[1, j]
        th = sub[(sub.family == "threshold_switch") & (sub.success == 1)].set_index("seed")
        ra = sub[(sub.family == "risk_aware") & (sub.success == 1)].set_index("seed")
        common = th.index.intersection(ra.index)
        diff = ra.loc[common, "energy_j"] - th.loc[common, "energy_j"]
        jitter = np.linspace(-.13, .13, len(diff))
        ax.scatter(jitter, diff.to_numpy(), color=COL["risk"], s=20, alpha=.78,
                   edgecolor="white", linewidth=.35)
        metric = stats["scenes"][scene]["paired_both_success"]["energy_j"]
        ci = metric["bootstrap_95pct_ci_mean_difference"]
        mean = metric["mean_difference"]
        ax.errorbar([0], [mean], yerr=[[mean-ci[0]], [ci[1]-mean]], fmt="D",
                    color=COL["ink"], ecolor=COL["ink"], capsize=4, markersize=4,
                    linewidth=1.2, zorder=5)
        ax.axhline(0, color="#777", linewidth=.7, linestyle="--")
        ax.set_xlim(-.23, .23); ax.set_xticks([])
        ax.set_ylabel("Risk-aware − threshold\nlegacy task-effort proxy (units)" if j == 0 else "")
        ax.set_title(f"Joint success $n$={len(common)}; mean {mean:.2f} "
                     f"[{ci[0]:.2f}, {ci[1]:.2f}]\n"
                     f"Wilcoxon $p$={metric['p_two_sided']:.4g}", fontsize=7.0)
        ax.grid(axis="y", alpha=.16)
        panel(ax, "c" if j == 0 else "d", x=.01, y=.91)
    # A shared lower-panel scale prevents visual exaggeration across scenes.
    lower_values = []
    for scene in ("scene_b", "scene_c"):
        sub = runs[runs.scene == scene]
        th = sub[(sub.family == "threshold_switch") & (sub.success == 1)].set_index("seed")
        ra = sub[(sub.family == "risk_aware") & (sub.success == 1)].set_index("seed")
        common = th.index.intersection(ra.index)
        lower_values.extend((ra.loc[common, "energy_j"] - th.loc[common, "energy_j"]).tolist())
        lower_values.extend(stats["scenes"][scene]["paired_both_success"]["energy_j"]
                            ["bootstrap_95pct_ci_mean_difference"])
    pad = 3.0
    lo, hi = min(lower_values) - pad, max(lower_values) + pad
    for ax in axes[1, :]:
        ax.set_ylim(lo, hi)
    fig.subplots_adjust(top=.94, hspace=.53, wspace=.35, bottom=.08)
    save(fig, "fig03_f1_heldout_success_conditional_effort")

    paired_rows = []
    for scene in ("scene_b", "scene_c"):
        for metric_name, metric in stats["scenes"][scene]["paired_both_success"].items():
            if isinstance(metric, dict) and "mean_difference" in metric:
                paired_rows.append({
                    "scene": scene, "metric": metric_name, "n_joint_success": metric["n"],
                    "mean_difference_risk_minus_threshold": metric["mean_difference"],
                    "median_difference": metric["median_difference"],
                    "bootstrap_ci_low": metric["bootstrap_95pct_ci_mean_difference"][0],
                    "bootstrap_ci_high": metric["bootstrap_95pct_ci_mean_difference"][1],
                    "wilcoxon_p": metric["p_two_sided"],
                    "matched_rank_biserial": metric["matched_rank_biserial"],
                })
    pd.DataFrame(paired_rows).to_csv(TAB / "f1_joint_success_paired_statistics.csv", index=False)


def cvar_mechanism() -> None:
    rows = pd.read_csv(CVAR / "sampled_decision_boundary_comparison.csv")
    p_fail = np.linspace(0, .40, 401)
    nominal, loss, alpha = 10.0, 60.0, .8
    q = 1 - alpha
    cvar = np.where(p_fail < q, nominal + p_fail / q * (loss - nominal), loss)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))
    ax = axes[0]
    ax.plot(p_fail, cvar, color=COL["risk"], linewidth=2.0)
    ax.axvline(q, color=COL["unsafe"], linestyle="--", linewidth=.9)
    ax.text(q + .008, 55, "$1-\\alpha=0.2$", fontsize=6.5, color=COL["unsafe"])
    ax.set(xlabel="Surrogate mixing probability $f$", ylabel="Two-point CVaR score",
           title="Analytic Bernoulli tail-risk core")
    ax.grid(alpha=.17); panel(ax, "a")
    ax = axes[1]
    for pos, scene, color in [(1, "scene_b", "#8DA0CB"), (2, "scene_c", "#66C2A5")]:
        values = rows.loc[rows.scene == scene, "local_raw_fail_critical"].to_numpy()
        # Descriptive summary only: range, interquartile range, and median.
        lo, q1, med, q3, hi = np.quantile(values, [0, .25, .5, .75, 1])
        ax.plot([pos, pos], [lo, hi], color="#777", linewidth=.8)
        ax.add_patch(mpatches.Rectangle((pos-.18, q1), .36, q3-q1,
                                       facecolor=color, edgecolor=COL["ink"], linewidth=.7))
        ax.plot([pos-.18, pos+.18], [med, med], color=COL["ink"], linewidth=1.2)
        ax.text(pos, hi + .015, f"{len(values):,} logged rows", ha="center", fontsize=6.0)
    ax.axhline(0, color="#777", linewidth=.7, linestyle="--")
    ax.axhspan(-.225, 0, color=COL["air"], alpha=.035, zorder=0)
    ax.text(2.48, -.205, "air-score side at\nzero raw risk", ha="right", va="bottom",
            fontsize=6.0, color="#555")
    ax.set_xlim(.45, 2.55); ax.set_ylim(-.225, .09)
    ax.set_xticks([1, 2], ["Scene B", "Scene C"])
    ax.set_ylabel("Implied local raw-failure boundary")
    ax.set_title("Historical Stage 0 local boundaries ($w_{cvar}=0.75$)")
    ax.grid(axis="y", alpha=.16); panel(ax, "b")
    fig.subplots_adjust(top=.88, bottom=.18, wspace=.31)
    save(fig, "fig04_cvar_local_boundary_mechanism")


def rotors_timeline() -> None:
    trace = pd.read_csv(E1R2 / "R2_PAPER_TRACE_10HZ.csv")
    events = pd.read_csv(E1R2 / "R2_EVENT_TIMELINE.csv")
    data = trace[(trace.sim_time_s >= trace.sim_time_s.min()) &
                 (trace.sim_time_s <= 84.5)].copy()

    phase_map = {
        "GROUND_DRIVE": "GROUND", "GROUND_REACQUIRE": "GROUND",
        "TAKEOFF_REPOSITION": "REPOS",
        "TAKEOFF_REPOSITION_FAILED": "REPOS",
        "PRE_TAKEOFF_BRAKE": "TAKEOFF", "RELEASE_AND_ARM": "TAKEOFF",
        "TAKEOFF_VERTICAL": "TAKEOFF",
        "TAKEOFF_CLEARANCE_CONFIRM": "TAKEOFF",
        "AIR_HORIZONTAL_TRACK": "AIR",
        "LAND_VERTICAL": "LAND", "TOUCHDOWN_SETTLE": "LAND",
        "DISARM_CONFIRM": "LAND",
    }
    phase_colors = {
        "GROUND": "#D9D9D9", "REPOS": "#E8B66F", "TAKEOFF": "#75B798",
        "AIR": "#6BAED6", "LAND": "#B39DDB",
    }
    t_start, t_end = float(data.sim_time_s.min()), float(data.sim_time_s.max())
    done_event = events.loc[events.event.eq("mission_state:DONE")].iloc[0]
    done_t = float(done_event.sim_time_s)

    # Exact categorical intervals come from bag-derived handoff event times.
    handoffs = events.loc[events.event.str.startswith("handoff:", na=False),
                          ["sim_time_s", "event"]].copy()
    handoffs["state"] = handoffs.event.str.split(":", n=1).str[1].map(phase_map)
    handoffs = handoffs.dropna(subset=["state"]).sort_values("sim_time_s")
    prior = handoffs[handoffs.sim_time_s <= t_start]
    current_state = prior.iloc[-1].state if len(prior) else "GROUND"
    changes = [(t_start, current_state)]
    for row in handoffs[(handoffs.sim_time_s > t_start) &
                        (handoffs.sim_time_s < t_end)].itertuples():
        if row.state != changes[-1][1]:
            changes.append((float(row.sim_time_s), row.state))
    intervals = [(time_s, changes[i + 1][0] if i + 1 < len(changes) else t_end,
                  state) for i, (time_s, state) in enumerate(changes)]

    fig = plt.figure(figsize=(7.2, 5.0), facecolor="white")
    outer = fig.add_gridspec(2, 1, height_ratios=(1.18, 1.0), hspace=.56,
                             left=.09, right=.91, top=.90, bottom=.10)
    timeline = outer[0].subgridspec(2, 1, height_ratios=(.13, .87), hspace=.035)
    ax_state = fig.add_subplot(timeline[0])
    ax = fig.add_subplot(timeline[1], sharex=ax_state)

    for t0, t1, state in intervals:
        ax_state.broken_barh([(t0, t1 - t0)], (0, 1),
                             facecolors=phase_colors[state],
                             edgecolors="white", linewidth=.35)
    ax_state.axvline(done_t, color="#202020", linestyle="--", linewidth=.75,
                     dashes=(3, 2), zorder=4)
    ax_state.set(xlim=(t_start, t_end), ylim=(0, 1))
    ax_state.set_yticks([])
    ax_state.tick_params(axis="x", which="both", bottom=False, labelbottom=False)
    for spine in ax_state.spines.values():
        spine.set_visible(False)
    ax_state.text(-.047, .5, "Mode", transform=ax_state.transAxes,
                  ha="right", va="center", fontsize=6.5, color="#444")
    state_handles = [mpatches.Patch(facecolor=phase_colors[state], edgecolor="none",
                                    label=state)
                     for state in ("GROUND", "REPOS", "TAKEOFF", "AIR", "LAND")]
    ax_state.legend(handles=state_handles, ncol=5, frameon=False,
                    loc="lower center", bbox_to_anchor=(.5, 1.02),
                    handlelength=1.15, handleheight=.65,
                    columnspacing=1.15, handletextpad=.35, fontsize=6.2)
    ax_state.text(-.075, 1.35, "(a)", transform=ax_state.transAxes,
                  ha="left", va="bottom", fontsize=8.5, weight="bold",
                  color=COL["ink"])

    altitude_color = "#0072B2"
    motor_color = "#D55E00"
    ax.plot(data.sim_time_s, data.z_m, color=altitude_color, linewidth=1.25,
            zorder=3)
    ax.set(xlim=(t_start, t_end), ylim=(0, 1.55),
           xlabel="Simulation time (s)", ylabel="Altitude (m)")
    ax.set_yticks([0, .5, 1.0, 1.5])
    ax.tick_params(axis="y", colors=altitude_color)
    ax.yaxis.label.set_color(altitude_color)
    ax.grid(axis="y", color="#D8D8D8", linewidth=.5, alpha=.65)
    ax.set_axisbelow(True)

    ax_motor = ax.twinx()
    ax_motor.plot(data.sim_time_s, data.actual_motor_mean_abs_rad_s,
                  color=motor_color, linewidth=.9, alpha=.92, zorder=2)
    ax_motor.set_ylim(0, 820)
    ax_motor.set_yticks([0, 200, 400, 600, 800])
    ax_motor.set_ylabel("Mean rotor speed (rad s$^{-1}$)", color=motor_color)
    ax_motor.tick_params(axis="y", colors=motor_color)
    ax_motor.spines["top"].set_visible(False)
    ax.axvline(done_t, color="#202020", linestyle="--", linewidth=.8,
               dashes=(3, 2), zorder=5)
    ax.annotate("DONE", xy=(done_t, 1.0), xycoords=ax.get_xaxis_transform(),
                xytext=(-3, -5), textcoords="offset points", ha="right",
                va="top", fontsize=6.4, weight="bold", color="#202020")

    ax_map = fig.add_subplot(outer[1])
    world_path = SRC / "rescue_worlds" / "worlds" / "disaster_scene_a.world"
    root = ET.parse(world_path).getroot()
    obstacle_handles = []
    for model in root.findall(".//world/model"):
        pose_text = model.findtext("pose", default="0 0 0 0 0 0")
        pose = [float(v) for v in pose_text.split()]
        for collision in model.findall("./link/collision"):
            size_text = collision.findtext("./geometry/box/size")
            if not size_text:
                continue
            sx, sy, _ = [float(v) for v in size_text.split()]
            collision_pose = [float(v) for v in
                              collision.findtext("pose", default="0 0 0 0 0 0").split()]
            cx, cy = pose[0] + collision_pose[0], pose[1] + collision_pose[1]
            patch = mpatches.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                       facecolor="#D0D0D0", edgecolor="#5C5C5C",
                                       linewidth=.65, zorder=1)
            ax_map.add_patch(patch)
            obstacle_handles.append(patch)

    phases = data.handoff_state.map(phase_map).fillna("GROUND")
    ground_like = phases.isin(["GROUND", "REPOS"])
    air_like = phases.isin(["TAKEOFF", "AIR", "LAND"])
    ground_line, = ax_map.plot(data.x_m.where(ground_like), data.y_m.where(ground_like),
                               color="#8C5A2B", linewidth=1.35,
                               linestyle=(0, (4, 2.2)), label="Ground / reposition",
                               zorder=3)
    air_line, = ax_map.plot(data.x_m.where(air_like), data.y_m.where(air_like),
                            color=altitude_color, linewidth=1.45,
                            label="Takeoff / air / land", zorder=4)

    start = data.iloc[0]
    start_mark = ax_map.scatter(start.x_m, start.y_m, s=30, marker="o",
                                facecolor="white", edgecolor="#202020",
                                linewidth=.9, zorder=6, label="Start")

    event_specs = [
        ("requested_mode:TAKEOFF", 10.000, "1", (3.15, .50)),
        ("target_reachability:unverified_cross_sortie", 30.023, "2", (5.75, 1.68)),
        ("approved_takeoff_pose", 50.540, "3", (5.85, .20)),
        ("mission_state:DONE", 80.000, "4", (1.10, -.52)),
    ]
    for event_name, target_time, number, label_xy in event_specs:
        candidates = events[events.event.eq(event_name)].copy()
        row = candidates.iloc[(candidates.sim_time_s - target_time).abs().argsort()[:1]].iloc[0]
        ax_map.scatter(row.x_m, row.y_m, s=26, facecolor="white",
                       edgecolor="#202020", linewidth=.75, zorder=7)
        ax_map.annotate(number, xy=(row.x_m, row.y_m), xycoords="data",
                        xytext=label_xy, textcoords="data", ha="center", va="center",
                        fontsize=6.5, weight="bold", color="#202020",
                        bbox=dict(boxstyle="circle,pad=.20", facecolor="white",
                                  edgecolor="#202020", linewidth=.7),
                        arrowprops=dict(arrowstyle="-", color="#707070", lw=.65,
                                        shrinkA=5, shrinkB=4), zorder=8)

    ax_map.set(xlim=(.35, 10.55), ylim=(-1.42, 1.82),
               xlabel="x (m)", ylabel="y (m)")
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.grid(color="#D8D8D8", linewidth=.5, alpha=.55)
    ax_map.set_axisbelow(True)
    ax_map.text(-.075, 1.14, "(b)", transform=ax_map.transAxes,
                ha="left", va="bottom", fontsize=8.5, weight="bold",
                color=COL["ink"])
    obstacle_proxy = mpatches.Patch(facecolor="#D0D0D0", edgecolor="#5C5C5C",
                                    label="Obstacle footprint")
    ax_map.legend(handles=[ground_line, air_line, start_mark, obstacle_proxy],
                  frameon=False, loc="upper center", bbox_to_anchor=(.54, 1.14),
                  ncol=4, columnspacing=1.15, handlelength=2.2,
                  handletextpad=.45, fontsize=6.3)

    save(fig, "fig05_rotors_representative_timeline_context")
    events.to_csv(TAB / "rotors_r2_full_mission_events.csv", index=False)

def d2_supplement() -> None:
    trials = pd.read_csv(D2 / "calibration_trial_summary.csv")
    fig, ax = plt.subplots(figsize=(3.35, 3.35))
    for split, marker, color in (("calibration", "o", COL["rotors"]),
                                 ("heldout", "^", COL["unsafe"])):
        sub = trials[trials.split == split]
        ax.scatter(sub.mechanical_energy_j / 1000, sub.predicted_mechanical_energy_j / 1000,
                   marker=marker, s=34, color=color, edgecolor="white", linewidth=.5,
                   label=f"{split} (n={len(sub)})")
    lo = min(trials.mechanical_energy_j.min(), trials.predicted_mechanical_energy_j.min()) / 1000 - .4
    hi = max(trials.mechanical_energy_j.max(), trials.predicted_mechanical_energy_j.max()) / 1000 + .4
    ax.plot([lo, hi], [lo, hi], "--", color="#555", linewidth=.8, label="Identity")
    ax.set(xlim=(lo, hi), ylim=(lo, hi), xlabel="Measured shaft-mechanical proxy (kJ)",
           ylabel="Predicted shaft-mechanical proxy (kJ)")
    ax.text(.04, .96, "held-out MAE 245.2 J\nmax underprediction 847.7 J\n"
            "operational margin rounded to 900 J", transform=ax.transAxes,
            ha="left", va="top", fontsize=6.3)
    ax.legend(frameon=False, loc="lower right")
    ax.grid(alpha=.17)
    save(fig, "figS1_d2_measured_predicted_mechanical_proxy")


def tables_and_sources() -> None:
    stats = json.loads((F1 / "heldout_paired_statistics.json").read_text())
    rows = []
    for scene in ("scene_b", "scene_c"):
        block = stats["scenes"][scene]
        energy = block["paired_both_success"]["energy_j"]
        rows.append({
            "scene": scene[-1].upper(),
            "threshold_success": block["success"]["threshold_switch"],
            "risk_success": block["success"]["risk_aware"],
            "fixed_success": block["success"]["fixed_reference"],
            "mcnemar_p": block["mcnemar"]["exact_two_sided_p"],
            "joint_success_n": block["both_success_n"],
            "energy_mean_difference_j": energy["mean_difference"],
            "energy_ci_low_j": energy["bootstrap_95pct_ci_mean_difference"][0],
            "energy_ci_high_j": energy["bootstrap_95pct_ci_mean_difference"][1],
            "energy_wilcoxon_p": energy["p_two_sided"],
            "energy_rank_biserial": energy["matched_rank_biserial"],
        })
    pd.DataFrame(rows).to_csv(TAB / "f1_primary_results.csv", index=False)

    pd.DataFrame([
        {"condition": "Nominal Scene D", "unguarded_irreversible": "4/20",
         "guarded_irreversible": "0/20", "exact_mcnemar_p": .125,
         "mission_success_unguarded": "0/20", "mission_success_guarded": "0/20"},
        {"condition": "Forced-flight stress", "unguarded_irreversible": "20/20",
         "guarded_irreversible": "0/20", "exact_mcnemar_p": 1.9073486328125e-6,
         "mission_success_unguarded": "0/20", "mission_success_guarded": "0/20"},
    ]).to_csv(TAB / "supervisor_outcomes.csv", index=False)

    sources = [
        ("Fig. 1a", "robot top-view geometry", "uav_v4/urdf/uav_v4_rotors_native.urdf.xacro"),
        ("Fig. 1b-c", "Scene B/C routes, risk fields, and solid obstacles",
         "rescue_mission/config/scenes/scene_{b,c}.yaml; rescue_worlds/worlds/disaster_scene_{b,c}.world"),
        ("Fig. 1d", "audited decision and evidence contracts", "POST_F1_EVIDENCE_MANIFEST.md; Phase F0/D2/E1 reports"),
        ("Fig. 2", "Stage 0 Scene B permanent/recoverable summaries", "rescue_mission/results/summary_regime_scene_b_{permanent,recoverable}_*.csv"),
        ("Fig. 3", "F1 held-out runs and paired statistics", "rescue_mission/results/phase_f1_fair_tuning_20260821/{heldout_run_summary.csv,heldout_paired_statistics.json}"),
        ("Fig. 4", "two-point formula and frozen decision rows", "rescue_worlds/results/phase_e1a_contact_20260731/readonly_audit_cvar/sampled_decision_boundary_comparison.csv"),
        ("Fig. 5", "post-freeze RotorS Scene A full-mission extension and SDF geometry", "rescue_worlds/results/phase_e1a_r2_full_scene_a_20260828/scene_a_frozen/run_1/{R2_PAPER_TRACE_10HZ.csv,R2_EVENT_TIMELINE.csv,R2_BAG_AUDIT.json}; disaster_scene_a.world"),
        ("Fig. S1", "D2 calibration/held-out mechanical proxy", "rescue_worlds/results/uav_v4_rotors_energy_d2_20260719/calibration_trial_summary.csv"),
    ]
    pd.DataFrame(sources, columns=["artifact", "content", "frozen_source"]).to_csv(
        EVID / "figure_source_map.csv", index=False)


def main() -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    TAB.mkdir(parents=True, exist_ok=True)
    EVID.mkdir(parents=True, exist_ok=True)
    configure_style()
    system_and_scenes()
    stage0_regime()
    f1_heldout()
    cvar_mechanism()
    rotors_timeline()
    d2_supplement()
    tables_and_sources()
    print(f"Generated figure artifacts in {FIG}")


if __name__ == "__main__":
    main()
