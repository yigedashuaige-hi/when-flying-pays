#!/usr/bin/env python3
"""
Run repeated Phase 1 experiments and summarize results.

Example:
    rosrun rescue_mission run_phase1_experiments.py --runs 5
    rosrun rescue_mission run_phase1_experiments.py --strategies rule fixed_switch --runs 3
    rosrun rescue_mission run_phase1_experiments.py --strategies rule_energy_shield --runs 1
"""
import argparse
import datetime
import os
import signal
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_STRATEGIES = ["ground_only", "air_preferred", "fixed_switch", "rule"]


def stop_process_group(proc):
    if proc.poll() is not None:
        return proc.returncode

    for sig, wait_sec in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return proc.poll()
        try:
            return proc.wait(timeout=wait_sec)
        except subprocess.TimeoutExpired:
            continue

    return proc.poll()


def run_command(cmd, timeout_sec):
    proc = subprocess.Popen(cmd, start_new_session=True)
    try:
        return proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        return stop_process_group(proc)
    except KeyboardInterrupt:
        stop_process_group(proc)
        raise


def resolve_seeds(args):
    if args.seeds:
        return list(args.seeds)
    if args.num_seeds and args.num_seeds > 0:
        return [args.base_seed + i for i in range(args.num_seeds)]
    # Legacy: a single deterministic run (seed -1 disables the randomized terrain).
    return [-1]


def run_experiments(args):
    seeds = resolve_seeds(args)
    runs_dir = args.runs_dir or os.path.join(PKG_DIR, "results", "runs")
    total = len(args.strategies) * len(seeds) * args.runs
    current = 0
    batch_start = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    interrupted = False
    try:
        # Seed is the outer loop after strategy so that, for a fixed seed, every
        # strategy faces the identical randomized environment (Common Random
        # Numbers / fair paired comparison).
        for strategy in args.strategies:
            for seed in seeds:
                for run_idx in range(1, args.runs + 1):
                    current += 1
                    print(
                        f"\n[{current}/{total}] strategy={strategy} seed={seed} run={run_idx}/{args.runs}",
                        flush=True,
                    )
                    cmd = [
                        "roslaunch",
                        "rescue_mission",
                        "rescue_phase1_demo.launch",
                        f"strategy:={strategy}",
                        f"scene:={args.scene}",
                        f"seed:={seed}",
                        f"mission_timeout:={args.mission_timeout}",
                        f"runs_dir:={runs_dir}",
                        f"ablation:={args.ablation}",
                        f"heteroscedastic:={'true' if args.heteroscedastic else 'false'}",
                        f"initial_battery:={args.initial_battery}",
                        f"switch_tau:={args.switch_tau}",
                        f"stuck_model:={args.stuck_model}",
                        f"fly_k1:={args.fly_k1}",
                        f"gui:={'true' if args.gui else 'false'}",
                        f"headless:={'false' if args.gui else 'true'}",
                        "paused:=false",
                    ]
                    code = run_command(cmd, args.duration)
                    print(f"  finished with code {code}", flush=True)
                    time.sleep(args.cooldown)
    except KeyboardInterrupt:
        interrupted = True
        print("\nInterrupted: stopped active roslaunch process group.", flush=True)

    summary_path = args.summary_path or os.path.join(PKG_DIR, "results", "summary.csv")
    summarize_cmd = [
        sys.executable,
        os.path.join(SCRIPT_DIR, "summarize_results.py"),
        runs_dir,
        summary_path,
        "--min-duration",
        str(args.min_duration),
        "--scene",
        args.scene,
        "--since",
        batch_start,
    ]
    print("\nSummarizing results...", flush=True)
    code = subprocess.call(summarize_cmd)
    if code == 0 and args.plot:
        stem = os.path.splitext(os.path.basename(summary_path))[0]
        output_dir = args.figure_dir or os.path.join(os.path.dirname(summary_path), f"figures_{stem}")
        plot_cmd = [
            sys.executable,
            os.path.join(SCRIPT_DIR, "plot_results.py"),
            summary_path,
            "--output-dir",
            output_dir,
        ]
        print("\nGenerating figures...", flush=True)
        code = subprocess.call(plot_cmd)
    return 130 if interrupted else code


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--strategies", nargs="+", default=DEFAULT_STRATEGIES)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--scene", default="scene_a")
    # Seedable randomized environment (paper-hardening). Provide explicit --seeds
    # or --num-seeds (with --base-seed) for paired multi-seed batches. Omit both
    # for the legacy single deterministic run (seed -1).
    parser.add_argument("--seeds", nargs="*", type=int, default=None)
    parser.add_argument("--num-seeds", type=int, default=0)
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--mission-timeout", type=float, default=120.0)
    parser.add_argument("--ablation", default="none")
    parser.add_argument("--heteroscedastic", action="store_true",
                        help="Calibrated perception: p_trav noise scale = u_trav (heteroscedastic).")
    parser.add_argument("--initial-battery", type=float, default=-1.0,
                        help="Override initial_battery fraction (pilot battery levels); -1 = scene default.")
    parser.add_argument("--switch-tau", type=float, default=-1.0,
                        help="threshold_switch tau (fly iff 1-p_trav > tau); -1 = default.")
    parser.add_argument("--stuck-model", default="permanent",
                        help="Ground stuck model axis: permanent | recoverable.")
    parser.add_argument("--fly-k1", type=float, default=-1.0,
                        help="Flight-cost axis: logger fly_k1 energy coefficient; -1 = default.")
    parser.add_argument("--duration", type=float, default=75.0)
    parser.add_argument("--min-duration", type=float, default=30.0)
    parser.add_argument("--cooldown", type=float, default=2.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--runs-dir", default="")
    parser.add_argument("--summary-path", default="")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--figure-dir", default="")
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(run_experiments(parse_args()))
