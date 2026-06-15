#!/usr/bin/env python3
"""
Visualize gradient descent optimization results from history.csv.

Usage:
    python3 scripts/plot_optimization.py --history runs/gradient_descent/history.csv
    python3 scripts/plot_optimization.py --run-dir runs/gradient_descent
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd


def parse_args():
    parser = argparse.ArgumentParser(description="Plot gradient descent optimization results")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--history", help="Path to history.csv")
    group.add_argument("--run-dir", help="Path to optimizer output directory (contains history.csv)")
    parser.add_argument("--output", help="Save figure to this path instead of showing interactively")
    return parser.parse_args()


def load_history(history_path: Path) -> pd.DataFrame:
    df = pd.read_csv(history_path)
    df["reachedT80"] = df["reachedT80"].astype(str).str.lower().map({"true": True, "false": False})
    return df


def load_summary(run_dir: Path) -> dict | None:
    summary_path = run_dir / "summary.json"
    if summary_path.exists():
        with open(summary_path, encoding="utf-8") as f:
            return json.load(f)
    return None


def param_columns(df: pd.DataFrame) -> list[str]:
    grad_cols = {c[5:] for c in df.columns if c.startswith("grad_")}
    return [c for c in df.columns if c in grad_cols]


def gradient_norm(df: pd.DataFrame, params: list[str]) -> pd.Series:
    grad_cols = [f"grad_{p}" for p in params]
    return np.sqrt((df[grad_cols] ** 2).sum(axis=1))


def color_by_status(df: pd.DataFrame):
    return ["tab:green" if r else "tab:red" for r in df["reachedT80"]]


def plot(df: pd.DataFrame, summary: dict | None, output_path: str | None):
    params = param_columns(df)
    n_params = len(params)
    iterations = df["iteration"].values

    colors = color_by_status(df)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Gradient Descent Optimization", fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # --- Quality over iterations ---
    ax_quality = fig.add_subplot(gs[0, :])
    ax_quality.scatter(iterations, df["quality"], c=colors, s=30, zorder=3)
    ax_quality.plot(iterations, df["quality"], color="gray", linewidth=0.8, zorder=2)
    best_idx = df["quality"].idxmin()
    ax_quality.scatter(
        [df.loc[best_idx, "iteration"]],
        [df.loc[best_idx, "quality"]],
        s=120, marker="*", color="gold", edgecolors="black", linewidths=0.8, zorder=4,
        label=f"best quality = {df.loc[best_idx, 'quality']:.4f}",
    )

    max_time = df["simulatedTime"].max()
    ax_quality.axhline(max_time, color="tab:orange", linestyle="--", linewidth=0.8, label=f"maxSimulationTime = {max_time:.1f}")
    ax_quality.set_xlabel("Iteration")
    ax_quality.set_ylabel("Quality (lower = better)")
    ax_quality.set_title("Quality per iteration")
    ax_quality.legend(fontsize=8)

    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="tab:green", label="reached t80"),
        Patch(facecolor="tab:red", label="penalized"),
    ]
    ax_quality.legend(handles=legend_elements + ax_quality.get_legend_handles_labels()[0][1:],
                      fontsize=8, loc="upper right")

    # --- t80 over iterations (only successful runs) ---
    ax_t80 = fig.add_subplot(gs[1, 0])
    mask_t80 = df["reachedT80"]
    if mask_t80.any():
        ax_t80.scatter(df.loc[mask_t80, "iteration"], df.loc[mask_t80, "t80"],
                       color="tab:green", s=30)
        ax_t80.plot(df.loc[mask_t80, "iteration"], df.loc[mask_t80, "t80"],
                    color="tab:green", linewidth=0.8, alpha=0.6)
    ax_t80.set_xlabel("Iteration")
    ax_t80.set_ylabel("t80 (s)")
    ax_t80.set_title("t80 (successful runs only)")

    # --- Gradient norm over iterations ---
    ax_grad = fig.add_subplot(gs[1, 1])
    gnorm = gradient_norm(df, params)
    ax_grad.plot(iterations, gnorm, color="tab:purple", linewidth=1.0)
    ax_grad.scatter(iterations, gnorm, c=colors, s=20, zorder=3)
    ax_grad.set_xlabel("Iteration")
    ax_grad.set_ylabel("‖∇quality‖")
    ax_grad.set_title("Gradient norm")
    ax_grad.set_yscale("symlog", linthresh=1e-3)

    # --- Parameter trajectories ---
    ax_params = fig.add_subplot(gs[2, :])
    cmap = plt.get_cmap("tab10")
    for idx, param in enumerate(params):
        vals = df[param].values
        norm_vals = (vals - vals.min()) / (vals.max() - vals.min() + 1e-12)
        ax_params.plot(iterations, norm_vals, label=param, color=cmap(idx), linewidth=1.2)
        ax_params.scatter(iterations, norm_vals, color=cmap(idx), s=15, zorder=3)

    ax_params.set_xlabel("Iteration")
    ax_params.set_ylabel("Normalised value [0, 1]")
    ax_params.set_title("Parameter trajectories (each normalised to its own range)")
    ax_params.legend(fontsize=7, ncol=min(n_params, 5))

    # Annotate with summary stats if available
    if summary:
        info_lines = [
            f"iterations: {summary['iterations']}",
            f"lr: {summary['learningRate']}",
            f"fd_step: {summary['finiteDifferenceStep']}",
            f"total evals: {summary['totalEvaluations']}",
            f"best quality: {summary['quality']['bestQuality']:.4f}",
            f"best status: {summary['bestStatus']}",
        ]
        fig.text(
            0.99, 0.98, "\n".join(info_lines),
            ha="right", va="top", fontsize=7.5,
            family="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", edgecolor="gray", alpha=0.85),
        )

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved to {output_path}")
    else:
        plt.show()


def main():
    args = parse_args()

    if args.run_dir:
        run_dir = Path(args.run_dir)
        history_path = run_dir / "history.csv"
        summary = load_summary(run_dir)
    else:
        history_path = Path(args.history)
        run_dir = history_path.parent
        summary = load_summary(run_dir)

    if not history_path.exists():
        raise FileNotFoundError(f"history.csv not found: {history_path}")

    df = load_history(history_path)
    print(f"Loaded {len(df)} iterations from {history_path}")

    plot(df, summary, args.output)


if __name__ == "__main__":
    main()
