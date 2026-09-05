#!/usr/bin/env python3
"""Command-line entry point for a configured swarm experiment.

Usage:
    python run_experiment.py                          # default: v3_two
    python run_experiment.py --config v3_two
    python run_experiment.py --yes --no-plot --no-record

The script runs preflight checks (mocap validity, minimum separation,
initial formation-error table) before any wheel command is issued.
``--yes`` only skips the final ENTER confirmation; the hard checks cannot
be skipped.
"""

from __future__ import annotations

import argparse
import importlib
from dataclasses import replace

from swarm.application.bootstrap import build_control_loop
from swarm.domain.config import ExperimentConfig


def _load_config(name: str) -> ExperimentConfig:
    module_name = name if "." in name else "configs." + name
    module = importlib.import_module(module_name)
    config = getattr(module, "CONFIG", None)
    if not isinstance(config, ExperimentConfig):
        raise TypeError("%s.CONFIG must be an ExperimentConfig" % module_name)
    return config


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a configured swarm experiment")
    parser.add_argument(
        "--config",
        default="v3_two",
        help="configuration module, default: configs.v3_two",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the preflight ENTER confirmation (hard checks still apply)",
    )
    parser.add_argument("--no-plot", action="store_true", help="disable the live trajectory window")
    parser.add_argument("--no-record", action="store_true", help="disable local CSV recording")
    return parser.parse_args()
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the preflight ENTER confirmation (hard checks still apply)",
    )
    parser.add_argument("--no-plot", action="store_true", help="disable the live trajectory window")
    parser.add_argument("--no-record", action="store_true", help="disable local CSV recording")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    config = _load_config(args.config)
    telemetry = replace(
        config.runtime.telemetry,
        live_plot_enabled=config.runtime.telemetry.live_plot_enabled and not args.no_plot,
        record_enabled=config.runtime.telemetry.record_enabled and not args.no_record,
    )
    runtime = replace(config.runtime, telemetry=telemetry)
    build_control_loop(replace(config, runtime=runtime), assume_yes=args.yes).run()


if __name__ == "__main__":
    main()
