from pathlib import Path
from simple_scenario import Scenario
from tqdm import tqdm


def main() -> None:
    scenario_dir = Path(__file__).parent

    for scenario_file in tqdm(list(scenario_dir.glob("*.json")), desc="Generate GIFs"):
        scenario = Scenario.from_x(scenario_file)
        scenario.render_gif(scenario_dir)


if __name__ == "__main__":
    main()
