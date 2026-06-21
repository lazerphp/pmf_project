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


def step_norm(df: pd.DataFrame, params: list[str]) -> np.ndarray:
    if not params:
        return np.array([])

    values = df[params].to_numpy(dtype=float)
    if len(values) == 0:
        return np.array([])

    diffs = np.vstack([np.full((1, len(params)), np.nan), np.diff(values, axis=0)])
    return np.sqrt(np.nansum(diffs ** 2, axis=1))


def parameter_bounds(summary: dict | None) -> dict[str, tuple[float, float]]:
    if not summary or "optimizationParameters" not in summary:
        return {}

    return {
        parameter["name"]: (float(parameter["min"]), float(parameter["max"]))
        for parameter in summary["optimizationParameters"]
    }


def bound_hit_counts(df: pd.DataFrame, params: list[str], bounds: dict[str, tuple[float, float]]) -> np.ndarray:
    if not params or not bounds:
        return np.array([])

    counts = []
    for _, row in df.iterrows():
        hit_count = 0
        for param in params:
            lower_bound, upper_bound = bounds[param]
            value = float(row[param])
            if np.isclose(value, lower_bound, atol=1e-9) or np.isclose(value, upper_bound, atol=1e-9):
                hit_count += 1
        counts.append(hit_count)
    return np.array(counts, dtype=int)


def parameter_heatmap(df: pd.DataFrame, params: list[str], bounds: dict[str, tuple[float, float]]) -> np.ndarray:
    if not params:
        return np.empty((0, 0))

    columns = []
    for param in params:
        values = df[param].to_numpy(dtype=float)
        if param in bounds:
            lower_bound, upper_bound = bounds[param]
        else:
            lower_bound = float(np.min(values))
            upper_bound = float(np.max(values))

        span = upper_bound - lower_bound
        if abs(span) < 1e-12:
            normalized = np.full_like(values, 0.5, dtype=float)
        else:
            normalized = (values - lower_bound) / span
        columns.append(normalized)

    return np.array(columns)


def compact_param_label(param: str) -> str:
    label = param.replace("fieldModel.sources.sources[", "s[")
    label = label.replace("].strength", "].str")
    label = label.replace("].sigma", "].sig")
    return label


def color_by_status(df: pd.DataFrame):
    return ["tab:green" if r else "tab:red" for r in df["reachedT80"]]


def plot(df: pd.DataFrame, summary: dict | None, output_path: str | None):
    params = param_columns(df)
    n_params = len(params)
    iterations = df["iteration"].values
    bounds = parameter_bounds(summary)
    colors = color_by_status(df)
    best_so_far = df["quality"].cummin()
    quality_delta = df["quality"].diff()
    gnorm = gradient_norm(df, params)
    snorm = step_norm(df, params)
    hits = bound_hit_counts(df, params, bounds)
    heatmap = parameter_heatmap(df, params, bounds)
    start_quality = float(df.loc[df.index[0], "quality"])
    end_quality = float(df.loc[df.index[-1], "quality"])
    best_idx = df["quality"].idxmin()
    best_iteration = int(df.loc[best_idx, "iteration"])
    new_best_mask = df["quality"].eq(best_so_far)
    new_best_count = int(new_best_mask.sum())

    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Gradient Descent Optimization", fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # --- Quality and best-so-far ---
    ax_quality = fig.add_subplot(gs[0, :])
    ax_quality.scatter(iterations, df["quality"], c=colors, s=30, zorder=3)
    ax_quality.plot(iterations, df["quality"], color="gray", linewidth=0.8, alpha=0.7, zorder=2, label="current quality")
    ax_quality.plot(iterations, best_so_far, color="tab:blue", linewidth=2.0, zorder=3, label="best so far")
    ax_quality.scatter(
        df.loc[new_best_mask, "iteration"],
        df.loc[new_best_mask, "quality"],
        s=55,
        facecolors="none",
        edgecolors="tab:blue",
        linewidths=1.2,
        zorder=4,
        label="new best",
    )
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
    ax_quality.set_title("Current quality vs cumulative best")

    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="tab:green", label="reached t80"),
        Patch(facecolor="tab:red", label="penalized"),
    ]
    handles, labels = ax_quality.get_legend_handles_labels()
    ax_quality.legend(handles=legend_elements + handles, fontsize=8, loc="upper right")

    # --- Quality change per step ---
    ax_delta = fig.add_subplot(gs[1, 0])
    delta_colors = ["tab:gray"]
    delta_colors.extend("tab:green" if delta < 0 else "tab:red" for delta in quality_delta.iloc[1:])
    ax_delta.bar(iterations, quality_delta.fillna(0.0), color=delta_colors, alpha=0.8)
    ax_delta.axhline(0.0, color="black", linewidth=0.8)
    ax_delta.set_xlabel("Iteration")
    ax_delta.set_ylabel("Δquality from previous")
    ax_delta.set_title("Step outcome (negative is better)")

    # --- Gradient norm and step norm ---
    ax_grad = fig.add_subplot(gs[1, 1])
    grad_line = ax_grad.plot(iterations, gnorm, color="tab:purple", linewidth=1.2, label="||grad||")[0]
    ax_grad.scatter(iterations, gnorm, c=colors, s=20, zorder=3)
    ax_grad.set_xlabel("Iteration")
    ax_grad.set_ylabel("‖∇quality‖", color="tab:purple")
    ax_grad.tick_params(axis="y", labelcolor="tab:purple")
    ax_grad.set_title("Gradient size vs actual step size")
    ax_grad.set_yscale("symlog", linthresh=1e-3)

    ax_step = ax_grad.twinx()
    step_line = ax_step.plot(iterations, snorm, color="tab:blue", linewidth=1.2, label="||step||")[0]
    ax_step.set_ylabel("‖step‖", color="tab:blue")
    ax_step.tick_params(axis="y", labelcolor="tab:blue")
    ax_step.set_yscale("symlog", linthresh=1e-3)
    ax_grad.legend([grad_line, step_line], ["||grad||", "||step||"], fontsize=8, loc="upper right")

    # --- Parameters at bounds ---
    ax_bounds = fig.add_subplot(gs[2, 0])
    if hits.size > 0:
        ax_bounds.plot(iterations, hits, color="tab:brown", linewidth=1.4, marker="o", markersize=4)
        ax_bounds.fill_between(iterations, 0, hits, color="tab:brown", alpha=0.15)
        ax_bounds.set_ylim(0, max(n_params, int(hits.max())) + 0.5)
        ax_bounds.set_title("How many parameters are stuck on bounds")
        ax_bounds.set_ylabel("Count at min/max")
    else:
        ax_bounds.text(0.5, 0.5, "Bounds unavailable\n(summary.json required)", ha="center", va="center")
        ax_bounds.set_title("How many parameters are stuck on bounds")
    ax_bounds.set_xlabel("Iteration")

    # --- Parameter heatmap ---
    ax_params = fig.add_subplot(gs[2, 1])
    if heatmap.size > 0:
        image = ax_params.imshow(heatmap, aspect="auto", cmap="viridis", vmin=0.0, vmax=1.0)
        ax_params.axvline(best_iteration, color="white", linestyle="--", linewidth=1.0)
        ax_params.set_yticks(range(n_params))
        ax_params.set_yticklabels([compact_param_label(param) for param in params], fontsize=8)
        ax_params.set_xlabel("Iteration")
        ax_params.set_title("Parameter values normalized by declared bounds")
        cbar = fig.colorbar(image, ax=ax_params, fraction=0.046, pad=0.04)
        cbar.set_label("0=min bound, 1=max bound")
    else:
        ax_params.text(0.5, 0.5, "No parameter data", ha="center", va="center")
        ax_params.set_title("Parameter values normalized by declared bounds")

    # Annotate with summary stats if available
    bound_hits_text = "n/a"
    if hits.size > 0:
        bound_hits_text = f"{int((hits > 0).sum())}/{len(hits)} iters"

    if summary:
        info_lines = [
            f"iterations: {summary['iterations']}",
            f"lr: {summary['learningRate']}",
            f"fd_step: {summary['finiteDifferenceStep']}",
            f"total evals: {summary['totalEvaluations']}",
            f"start quality: {start_quality:.4f}",
            f"best quality: {summary['quality']['bestQuality']:.4f}",
            f"best iter: {best_iteration}",
            f"end quality: {end_quality:.4f}",
            f"new bests: {new_best_count}",
            f"bound hits: {bound_hits_text}",
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
    best_idx = df["quality"].idxmin()
    print(
        "Run summary: "
        f"start={df.loc[df.index[0], 'quality']:.4f}, "
        f"best={df.loc[best_idx, 'quality']:.4f} at iter={int(df.loc[best_idx, 'iteration'])}, "
        f"end={df.loc[df.index[-1], 'quality']:.4f}"
    )

    plot(df, summary, args.output)


if __name__ == "__main__":
    main()
