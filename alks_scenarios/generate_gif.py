"""Generate a GIF from an existing simple-scenario JSON config.

Usage:
	uv run alks_scenarios/generate_gif.py /path/to/scenario/configs/<scenario_name>.json
"""

from __future__ import annotations

import argparse

from pathlib import Path

from simple_scenario import Scenario


def _build_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(
		description=(
			"Load a simple-scenario JSON config and render only a GIF into the "
			"scenario's images directory."
		)
	)
	parser.add_argument(
		"scenario_json",
		type=Path,
		help="Path to the scenario JSON config in the configs directory.",
	)
	parser.add_argument(
		"--dpi",
		type=int,
		default=600,
		help="DPI used for rendering (default: 600).",
	)
	return parser


def generate_gif_from_config(scenario_json: str | Path, dpi: int = 600) -> Path:
	"""Generate a GIF from an existing scenario JSON config.

	The GIF is written to the sibling images directory of the scenario folder and
	named according to the input config filename, i.e. <scenario_name>.gif.
	"""
	scenario_json_path = Path(scenario_json)
	if not scenario_json_path.exists():
		msg = f"Scenario config does not exist: {scenario_json_path}"
		raise FileNotFoundError(msg)
	if scenario_json_path.suffix.lower() != ".json":
		msg = f"Expected a JSON config file, got: {scenario_json_path}"
		raise ValueError(msg)

	scenario = Scenario.from_x(scenario_json_path)

	scenario_dir = scenario_json_path.parent.parent
	images_dir = scenario_dir / "images"
	images_dir.mkdir(parents=True, exist_ok=True)

	rendered_gif_path = scenario.render_gif(images_dir, dpi=dpi)
	desired_gif_path = images_dir / f"{scenario_json_path.stem}.gif"

	if rendered_gif_path != desired_gif_path:
		rendered_gif_path.replace(desired_gif_path)

	return desired_gif_path


def main() -> None:
	parser = _build_parser()
	args = parser.parse_args()

	gif_path = generate_gif_from_config(args.scenario_json, dpi=args.dpi)
	print(gif_path)


if __name__ == "__main__":
	main()
