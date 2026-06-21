#!/usr/bin/env python3

import argparse
import csv
import json
import math
import re
import subprocess
import tempfile
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Gradient descent over simulation field parameters")
    parser.add_argument("--binary", default="./build/simulation", help="Path to the simulation binary")
    parser.add_argument(
        "--optimizer-config",
        default="examples/optimizer_config.json",
        help="Path to optimizer config JSON",
    )
    parser.add_argument(
        "--output-dir",
        default="runs/gradient_descent",
        help="Directory for optimizer artifacts",
    )
    return parser.parse_args()


def load_json(path):
    with open(path, "r", encoding="utf-8") as input_file:
        return json.load(input_file)


def write_json(path, payload):
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)
        output_file.write("\n")


def deep_merge(base, patch):
    if isinstance(base, dict) and isinstance(patch, dict):
        merged = dict(base)
        for key, value in patch.items():
            if key in merged:
                merged[key] = deep_merge(merged[key], value)
            else:
                merged[key] = value
        return merged

    return patch


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


def parameter_bounds_map(optimization_parameters):
    return {
        parameter["name"]: (float(parameter["min"]), float(parameter["max"]))
        for parameter in optimization_parameters
    }


def clamp_parameter_vector(parameter_values, optimization_parameters):
    bounds_by_name = parameter_bounds_map(optimization_parameters)
    clamped_values = {}
    adjustments = []

    for parameter_name, parameter_value in parameter_values.items():
        lower_bound, upper_bound = bounds_by_name[parameter_name]
        clamped_value = clamp(parameter_value, lower_bound, upper_bound)
        clamped_values[parameter_name] = clamped_value

        if not math.isclose(parameter_value, clamped_value, rel_tol=0.0, abs_tol=1e-12):
            adjustments.append(
                {
                    "name": parameter_name,
                    "original": parameter_value,
                    "clamped": clamped_value,
                    "min": lower_bound,
                    "max": upper_bound,
                }
            )

    return clamped_values, adjustments


def normalize_parameter_path(base_run_config, parameter_name):
    if "." in parameter_name or "[" in parameter_name:
        return parameter_name

    return f"fieldModel.{parameter_name}"


def parse_path_tokens(path):
    tokens = []
    for part in path.split("."):
        match = re.fullmatch(r"([^\[]+)(\[\d+\])*", part)
        if match is None:
            raise ValueError(f"Unsupported parameter path segment: {part}")

        tokens.append(match.group(1))
        indexes = re.findall(r"\[(\d+)\]", part)
        tokens.extend(int(index) for index in indexes)

    return tokens


def get_by_path(payload, path):
    current = payload
    for token in parse_path_tokens(path):
        current = current[token]
    return current


def set_by_path(payload, path, value):
    tokens = parse_path_tokens(path)
    current = payload

    for token in tokens[:-1]:
        current = current[token]

    current[tokens[-1]] = value


def current_parameter_vector(base_run_config, optimization_parameters):
    return {
        parameter["name"]: float(get_by_path(base_run_config, normalize_parameter_path(base_run_config, parameter["name"])))
        for parameter in optimization_parameters
    }


def apply_parameter_vector(base_run_config, parameter_values):
    run_config = json.loads(json.dumps(base_run_config))
    for parameter_name, parameter_value in parameter_values.items():
        set_by_path(run_config, normalize_parameter_path(base_run_config, parameter_name), parameter_value)
    return run_config


def extract_run_config(payload, source_path):
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object in {source_path}")

    if "runConfig" in payload:
        return json.loads(json.dumps(payload["runConfig"]))

    return json.loads(json.dumps(payload))


def resolve_base_run_config(optimizer_config, optimizer_config_path):
    base_run_config = {}

    run_config_path = optimizer_config.get("runConfigPath")
    if run_config_path is not None:
        resolved_path = (optimizer_config_path.parent / run_config_path).resolve()
        base_run_config = extract_run_config(load_json(resolved_path), resolved_path)

    if "runConfig" in optimizer_config:
        base_run_config = deep_merge(base_run_config, optimizer_config["runConfig"])

    if not base_run_config:
        raise ValueError("optimizer config must contain runConfigPath or runConfig")

    return base_run_config


def run_headless(binary_path, run_config):
    payload = {
        "apiVersion": 1,
        "runConfig": run_config,
    }

    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as input_file:
        json.dump(payload, input_file, indent=2)
        input_path = Path(input_file.name)

    try:
        completed = subprocess.run(
            [binary_path, "--headless", "--input-json", str(input_path)],
            check=True,
            capture_output=True,
            text=True,
        )
        return json.loads(completed.stdout)
    finally:
        input_path.unlink(missing_ok=True)


def compute_quality(result, quality_config):
    metrics = result["metrics"]
    mode = quality_config.get("mode", "t80_or_penalized_missing_fraction")

    if mode != "t80_or_penalized_missing_fraction":
        raise ValueError(f"Unsupported quality mode: {mode}")

    if "quality" not in metrics or metrics["quality"] is None:
        raise ValueError("Headless result does not contain metrics.quality")

    return float(metrics["quality"])


def compute_status(result):
    if result["metrics"]["reachedT80"]:
        return "reached_t80"

    return "penalized"


def evaluate(binary_path, base_run_config, parameter_values, quality_config):
    run_config = apply_parameter_vector(base_run_config, parameter_values)
    result = run_headless(binary_path, run_config)
    quality = compute_quality(result, quality_config)
    status = compute_status(result)

    # Keep the optimization-side quality explicit in saved artifacts even if
    # the C++ side later changes the built-in formula.
    result["optimizerEvaluation"] = {
        "quality": quality,
        "status": status,
        "qualityMode": quality_config.get("mode", "t80_or_penalized_missing_fraction"),
        "penaltyWeight": float(quality_config.get("penaltyWeight", 1.0)),
    }

    return {
        "runConfig": run_config,
        "result": result,
        "quality": quality,
        "status": status,
    }


def compute_gradient(
    binary_path,
    base_run_config,
    parameter_values,
    optimization_parameters,
    quality_config,
    finite_difference_step,
    clamp_to_range,
):
    gradient = {}
    bounds_by_name = parameter_bounds_map(optimization_parameters)
    evaluations = []

    for parameter in optimization_parameters:
        parameter_name = parameter["name"]
        lower_bound, upper_bound = bounds_by_name[parameter_name]

        plus_values = dict(parameter_values)
        minus_values = dict(parameter_values)

        plus_values[parameter_name] = parameter_values[parameter_name] + finite_difference_step
        minus_values[parameter_name] = parameter_values[parameter_name] - finite_difference_step

        if clamp_to_range:
            plus_values[parameter_name] = clamp(plus_values[parameter_name], lower_bound, upper_bound)
            minus_values[parameter_name] = clamp(minus_values[parameter_name], lower_bound, upper_bound)

        plus_evaluation = evaluate(binary_path, base_run_config, plus_values, quality_config)
        minus_evaluation = evaluate(binary_path, base_run_config, minus_values, quality_config)
        evaluations.extend([plus_evaluation, minus_evaluation])

        denominator = plus_values[parameter_name] - minus_values[parameter_name]
        if math.isclose(denominator, 0.0, abs_tol=1e-12):
            gradient[parameter_name] = 0.0
        else:
            gradient[parameter_name] = (plus_evaluation["quality"] - minus_evaluation["quality"]) / denominator

    return gradient, evaluations


def write_history_csv(path, history_rows, optimization_parameters):
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    field_names = [parameter["name"] for parameter in optimization_parameters]
    gradient_names = [f"grad_{parameter_name}" for parameter_name in field_names]

    with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "iteration",
                "status",
                "quality",
                "reachedT80",
                "t80",
                "uniqueTargetHits",
                "targetThresholdCount",
                "missingTargetFraction",
                "simulatedTime",
                "stepsExecuted",
                *field_names,
                *gradient_names,
            ]
        )

        for row in history_rows:
            writer.writerow(
                [
                    row["iteration"],
                    row["status"],
                    row["quality"],
                    row["metrics"]["reachedT80"],
                    row["metrics"]["t80"],
                    row["metrics"]["uniqueTargetHits"],
                    row["metrics"]["targetThresholdCount"],
                    row["metrics"]["missingTargetFraction"],
                    row["runtime"]["simulatedTime"],
                    row["runtime"]["stepsExecuted"],
                    *[row["parameters"][parameter_name] for parameter_name in field_names],
                    *[row["gradient"][f"grad_{parameter_name}"] for parameter_name in field_names],
                ]
            )


def main():
    args = parse_args()
    optimizer_config_path = Path(args.optimizer_config)
    optimizer_config = load_json(optimizer_config_path)

    optimizer = optimizer_config["optimizer"]
    if optimizer.get("method") != "gradient_descent":
        raise ValueError("optimizer.method must be 'gradient_descent'")

    base_run_config = resolve_base_run_config(optimizer_config, optimizer_config_path)
    quality_config = dict(base_run_config.get("quality", {}))
    if "quality" in optimizer_config:
        quality_config = deep_merge(quality_config, optimizer_config["quality"])
    base_run_config["quality"] = dict(quality_config)
    optimization_parameters = optimizer_config["optimizationParameters"]

    iterations = int(optimizer["iterations"])
    learning_rate = float(optimizer["learningRate"])
    finite_difference_step = float(optimizer["finiteDifferenceStep"])
    clamp_to_range = bool(optimizer.get("clampToRange", True))
    parameter_count = len(optimization_parameters)
    evaluations_per_iteration = 1 + 2 * parameter_count
    planned_total_evaluations = iterations * evaluations_per_iteration + 1

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    parameter_values = current_parameter_vector(base_run_config, optimization_parameters)
    initial_parameter_adjustments = []
    if clamp_to_range:
        parameter_values, initial_parameter_adjustments = clamp_parameter_vector(
            parameter_values,
            optimization_parameters,
        )
        if initial_parameter_adjustments:
            print("Clamped initial parameter values to declared optimization ranges:")
            for adjustment in initial_parameter_adjustments:
                print(
                    "  "
                    f"{adjustment['name']}: "
                    f"{adjustment['original']:.6f} -> {adjustment['clamped']:.6f} "
                    f"(range [{adjustment['min']:.6f}, {adjustment['max']:.6f}])"
                )
    best_evaluation = None
    history_rows = []
    total_evaluations = 0

    print(
        "Optimization plan: "
        f"iterations={iterations}, "
        f"parameters={parameter_count}, "
        f"evaluations_per_iteration={evaluations_per_iteration}, "
        f"planned_total_evaluations={planned_total_evaluations}"
    )

    for iteration in range(iterations):
        current_evaluation = evaluate(args.binary, base_run_config, parameter_values, quality_config)
        total_evaluations += 1

        if best_evaluation is None or current_evaluation["quality"] < best_evaluation["quality"]:
            best_evaluation = current_evaluation

        gradient, gradient_evaluations = compute_gradient(
            args.binary,
            base_run_config,
            parameter_values,
            optimization_parameters,
            quality_config,
            finite_difference_step,
            clamp_to_range,
        )
        total_evaluations += len(gradient_evaluations)

        history_rows.append(
            {
                "iteration": iteration,
                "status": current_evaluation["status"],
                "quality": current_evaluation["quality"],
                "metrics": dict(current_evaluation["result"]["metrics"]),
                "runtime": dict(current_evaluation["result"]["runtime"]),
                "parameters": dict(parameter_values),
                "gradient": {f"grad_{key}": value for key, value in gradient.items()},
            }
        )

        print(
            f"iter={iteration + 1}/{iterations} "
            f"evaluation={total_evaluations}/{planned_total_evaluations} "
            f"status={current_evaluation['status']} "
            f"quality={current_evaluation['quality']:.6f} "
            f"t80={current_evaluation['result']['metrics']['t80']}"
        )

        for parameter in optimization_parameters:
            parameter_name = parameter["name"]
            updated_value = parameter_values[parameter_name] - learning_rate * gradient[parameter_name]
            if clamp_to_range:
                updated_value = clamp(updated_value, float(parameter["min"]), float(parameter["max"]))
            parameter_values[parameter_name] = updated_value

    final_evaluation = evaluate(args.binary, base_run_config, parameter_values, quality_config)
    total_evaluations += 1
    if best_evaluation is None or final_evaluation["quality"] < best_evaluation["quality"]:
        best_evaluation = final_evaluation

    history_path = output_dir / "history.csv"
    best_config_path = output_dir / "best_run_config.json"
    best_result_path = output_dir / "best_result.json"
    summary_path = output_dir / "summary.json"

    write_history_csv(history_path, history_rows, optimization_parameters)
    write_json(best_config_path, {"apiVersion": 1, "runConfig": best_evaluation["runConfig"]})
    write_json(best_result_path, best_evaluation["result"])

    summary = {
        "optimizerConfigPath": args.optimizer_config,
        "binaryPath": args.binary,
        "method": optimizer["method"],
        "iterations": iterations,
        "learningRate": learning_rate,
        "finiteDifferenceStep": finite_difference_step,
        "clampToRange": clamp_to_range,
        "quality": {
            "mode": quality_config.get("mode", "t80_or_penalized_missing_fraction"),
            "penaltyWeight": float(quality_config.get("penaltyWeight", 1.0)),
            "bestQuality": best_evaluation["quality"],
        },
        "bestStatus": best_evaluation["status"],
        "optimizationParameters": optimization_parameters,
        "bestMetrics": best_evaluation["result"]["metrics"],
        "bestRuntime": best_evaluation["result"]["runtime"],
        "bestFieldModel": best_evaluation["runConfig"].get("fieldModel"),
        "totalEvaluations": total_evaluations,
        "initialParametersClamped": len(initial_parameter_adjustments),
        "initialParameterAdjustments": initial_parameter_adjustments,
        "artifacts": {
            "historyCsv": str(history_path),
            "bestRunConfig": str(best_config_path),
            "bestResult": str(best_result_path),
        },
    }
    write_json(summary_path, summary)

    print("\nBest result:")
    print(f"status={best_evaluation['status']}")
    print(f"quality={best_evaluation['quality']:.6f}")
    print(f"historyCsv={history_path}")
    print(f"bestRunConfig={best_config_path}")
    print(f"bestResult={best_result_path}")
    print(f"summary={summary_path}")
    print(f"totalEvaluations={total_evaluations}")


if __name__ == "__main__":
    main()
