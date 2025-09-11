"""
Create deceleration scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, p. 56
"""

from __future__ import annotations

import numpy as np

from pathlib import Path
from tqdm import tqdm

from simple_scenario import Scenario, EgoConfiguration, Vehicle
from simple_scenario.road import Road, StraightSegment


class DecelerationScenarioGenerator:
    def __init__(self, result_dir: str | Path) -> None:
        self._result_dir = Path(result_dir)
        self._result_dir.mkdir(exist_ok=True)

        # Parameters listed in Table 2 on page 43
        self._n_lanes = 2
        self._lane_width = 3.5
        self._min_road_length = 300
        self._ego_s0 = 100
        self._ego_t0 = 0
        self._ego_lanelet_id = 1001
        self._object_lanelet_id = 1001
        self._thw0 = 2

        # Varying values
        self._g = 9.81
        gx_min = 0
        gx_max = 1
        ve0_kmh_min = 10
        ve0_kmh_max = 60

        # Vary
        gx_step = 0.05
        ve0_kmh_step = 1
        self._gx_values = np.arange(gx_min, gx_max + gx_step, gx_step)
        self._ve0_kmh_values = np.arange(
            ve0_kmh_min, ve0_kmh_max + ve0_kmh_step, ve0_kmh_step
        )

    def create_all_scenarios(
        self,
        create_openx: bool = False,
        create_image: bool = False,
        create_gif: bool = False,
    ) -> None:
        # Create folders
        # Scenarios
        scenario_result_dir = self._result_dir / "scenarios"
        scenario_result_dir.mkdir(exist_ok=True)

        # Images
        scenario_image_dir = self._result_dir / "images"
        if create_image:
            scenario_image_dir.mkdir(exist_ok=True)

        # Gifs
        scenario_gif_dir = self._result_dir / "gifs"
        if create_gif:
            scenario_gif_dir.mkdir(exist_ok=True)

        n_scenarios = len(self._gx_values) * len(self._ve0_kmh_values)

        with tqdm(total=n_scenarios) as progress_bar:
            for gx in self._gx_values:
                for ve0_kmh in self._ve0_kmh_values:
                    self._create_single_scenario(
                        gx,
                        ve0_kmh,
                        scenario_result_dir,
                        scenario_gif_dir,
                        scenario_image_dir,
                        create_openx=create_openx,
                        create_image=create_image,
                    )
                    progress_bar.update(1)

    def _create_single_scenario(
        self,
        gx: float,
        ve0_kmh: float,
        scenario_result_dir: Path,
        scenario_gif_dir: Path,
        scenario_image_dir: Path,
        create_openx: bool = False,
        create_gif: bool = False,
        create_image: bool = False,
    ) -> None:
        ve0_kmh = int(np.round(ve0_kmh))
        scenario_name = f"deceleration_plot_ve0_{ve0_kmh}_gx_{gx:.2f}"

        vo0_kmh = ve0_kmh
        ve0 = ve0_kmh / 3.6
        vo0 = vo0_kmh / 3.6

        # Create simple scenario object
        ego_configuration = EgoConfiguration(
            self._ego_lanelet_id, self._ego_s0, self._ego_t0, ve0
        )

        # Object vehicle
        object_vehicle_length = Vehicle(0, 0, 0, 0, 0).length
        dx = ve0 * self._thw0
        object_s0 = (
            self._ego_s0 + dx + ego_configuration.length / 2 + object_vehicle_length / 2
        )
        object_a0 = -gx * self._g

        object_vehicle = Vehicle(
            0, self._object_lanelet_id, object_s0, self._ego_t0, vo0, object_a0
        )

        # Calculate road length
        if object_a0 == 0:
            object_deceleration_time = 0
        else:
            object_deceleration_time = vo0 / abs(object_a0)
        object_deceleration_time += 10
        scenario_duration = np.ceil(object_deceleration_time)

        ego_dist = vo0 * scenario_duration

        road_length = max(ego_dist, self._min_road_length) + self._ego_s0 + 100
        goal_position = self._ego_s0 + 0.75 * ego_dist
        road = Road(
            self._n_lanes,
            self._lane_width,
            segments=[StraightSegment(road_length)],
            goal_position=goal_position,
            speed_limit=60,
        )

        scenario = Scenario(
            scenario_name,
            road,
            ego_configuration,
            vehicles=[object_vehicle],
            duration=scenario_duration,
            check_feasibility=False,
        )

        scenario_config_dir = scenario_result_dir / "configs"
        scenario_config_dir.mkdir(exist_ok=True)
        scenario.save(scenario_config_dir)

        if create_openx:
            scenario_result_dir_openx = scenario_result_dir / "openx"
            scenario_result_dir_openx.mkdir(exist_ok=True)
            scenario.save(scenario_result_dir_openx, mode="openx")
        if create_image:
            scenario.render(scenario_image_dir, dpi=600)
        if create_gif:
            scenario.render_gif(scenario_gif_dir, dpi=600)


def generate_all_scenarios() -> None:
    result_dir = Path(__file__).parent / ".." / "results" / "annex3" / "deceleration"
    result_dir.mkdir(exist_ok=True, parents=True)

    scenario_generator = DecelerationScenarioGenerator(result_dir)
    scenario_generator.create_all_scenarios(create_openx=True, create_image=True)


if __name__ == "__main__":
    generate_all_scenarios()
