"""
Create cutin scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, pp. 45-52
"""

from __future__ import annotations

import multiprocessing as mp
import numpy as np

from loguru import logger
from pathlib import Path
from tqdm import tqdm

from simple_scenario import Scenario, EgoConfiguration, Vehicle
from simple_scenario.road import Road, StraightSegment


class CutInScenarioGenerator:
    def __init__(self, result_dir: str | Path) -> None:
        self._result_dir = Path(result_dir)
        self._result_dir.mkdir(exist_ok=True)

        # Parameters listed in Table 2 on page 43
        self._n_lanes = 2
        self._lane_width = 3.5
        self._min_road_length = 400
        self._ego_s0 = 100
        self._ego_t0 = 0
        self._ego_lanelet_id = 1001
        self._object_lanelet_id = 1000
        self._dy0 = 1.6

        # Varying values
        vy_min = 0
        vy_max = 3
        dx0_min = 0
        dx0_max = 60
        # Vary
        vy_step = 0.1
        dx0_step = 1
        self._vy_values = np.arange(vy_min + vy_step, vy_max + vy_step, vy_step)
        self._dx0_values = np.arange(dx0_min, dx0_max + dx0_step, dx0_step)

        # Parameters per plot
        ve0_kmh_values = [60, 60, 60, 60, 50, 50, 50, 50, 40, 40, 40, 30, 30, 20]
        dv0_values = [0, 20, 30, 40, 10, 20, 30, 40, 10, 20, 30, 10, 20, 10]

        n_plots = len(ve0_kmh_values)
        self._params_per_plot_no = {
            i + 1: {"ve0_kmh": ve0_kmh_values[i], "dv0_kmh": dv0_values[i]}
            for i in range(n_plots)
        }

    def create_all_scenarios(
        self,
        only_plot_no: int | None = None,
        create_openx: bool = False,
        create_image: bool = False,
        create_gif: bool = False,
    ) -> None:
        for plot_no, params in self._params_per_plot_no.items():
            if only_plot_no is not None and plot_no != only_plot_no:
                continue
            self._create_scenarios_of_plot_no(
                plot_no,
                **params,
                create_gif=create_gif,
                create_image=create_image,
                create_openx=create_openx,
            )

    def _create_scenarios_of_plot_no(
        self,
        plot_no: int,
        ve0_kmh: float,
        dv0_kmh: float,
        create_openx: bool = False,
        create_image: bool = False,
        create_gif: bool = False,
    ) -> None:
        # Create folders
        plot_no_result_dir = self._result_dir / f"plot_{plot_no:02d}"
        plot_no_result_dir.mkdir(exist_ok=True)

        # Scenarios
        scenario_result_dir = plot_no_result_dir / "scenarios"
        scenario_result_dir.mkdir(exist_ok=True)

        # Images
        scenario_image_dir = plot_no_result_dir / "images"
        if create_image:
            scenario_image_dir.mkdir(exist_ok=True)

        # Gifs
        scenario_gif_dir = plot_no_result_dir / "gifs"
        if create_gif:
            scenario_gif_dir.mkdir(exist_ok=True)

        vo0_kmh = ve0_kmh - dv0_kmh
        ve0 = ve0_kmh / 3.6
        vo0 = vo0_kmh / 3.6

        n_scenarios = len(self._dx0_values) * len(self._vy_values)

        with tqdm(total=n_scenarios) as progress_bar:
            for dx0 in self._dx0_values:
                for vy in self._vy_values:
                    self._create_single_scenario(
                        plot_no,
                        dx0,
                        vy,
                        ve0,
                        vo0,
                        scenario_result_dir,
                        scenario_gif_dir,
                        scenario_image_dir,
                        create_gif=create_gif,
                        create_image=create_image,
                        create_openx=create_openx,
                    )
                    progress_bar.update(1)

    def _create_single_scenario(
        self,
        plot_no: int,
        dx0: float,
        vy: float,
        ve0: float,
        vo0: float,
        scenario_result_dir: Path,
        scenario_gif_dir: Path,
        scenario_image_dir: Path,
        create_gif: bool = False,
        create_image: bool = False,
        create_openx: bool = False,
    ) -> None:
        ve0_kmh = ve0 * 3.6
        vo0_kmh = vo0 * 3.6
        ve0_kmh = int(np.round(ve0_kmh))
        dv0_kmh = int(np.round(ve0_kmh - vo0_kmh))
        scenario_name = f"cutin_plot_{plot_no:02d}_ve0_{ve0_kmh}_dv0_{dv0_kmh}_dx0_{dx0:.1f}_vy_{vy:.1f}"

        # Create simple scenario object
        ego_configuration = EgoConfiguration(
            self._ego_lanelet_id, self._ego_s0, self._ego_t0, ve0
        )

        # Calculate object_vehicle_t0 from dy0
        ego_width = ego_configuration.width
        dummy_vehicle = Vehicle(0, 0, 0, 0, 0)
        object_vehicle_width = dummy_vehicle.width
        object_vehicle_t0_from_ego_llt = ego_configuration.t0 - (
            self._dy0 + ego_width / 2 + object_vehicle_width / 2
        )
        object_vehicle_t0 = object_vehicle_t0_from_ego_llt + self._lane_width

        # Calculate object_vehicle_s0 from ego_s and dx0
        ego_length = ego_configuration.length
        object_vehicle_length = dummy_vehicle.length
        object_vehicle_s0 = (
            self._ego_s0 + ego_length / 2 + dx0 + object_vehicle_length / 2
        )

        # Calculate lc_duration
        object_vehicle_lc_duration = self._dy0 / vy
        object_vehicle = Vehicle(
            0,
            self._object_lanelet_id,
            object_vehicle_s0,
            object_vehicle_t0,
            vo0,
            lc_direction=1,
            lc_type="vy",
            lc_vy=vy,
        )
        scenario_duration = np.ceil(object_vehicle_lc_duration) + 10

        # Calculate road length
        ego_dist = ve0 * scenario_duration
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


def unpack_and_run(args_list: list) -> None:
    scenario_generator = CutInScenarioGenerator(args_list[0])
    scenario_generator.create_all_scenarios(
        only_plot_no=args_list[1], create_image=True, create_openx=True
    )


def generate_all_scenarios() -> None:
    result_dir = Path(__file__).parent / ".." / "results" / "annex3" / "cutin"
    result_dir.mkdir(exist_ok=True, parents=True)

    n_plots_in_regulation = 14

    n_workers = n_plots_in_regulation

    all_args_lists = [[result_dir, i + 1] for i in range(n_plots_in_regulation)]

    if n_workers == 1:
        for arg_list in all_args_lists:
            unpack_and_run(arg_list)
    else:
        with mp.Pool(n_workers) as pool:
            result_iterator = pool.imap(unpack_and_run, all_args_lists, chunksize=1)
            for _ in result_iterator:
                pass
    logger.info("done")


if __name__ == "__main__":
    generate_all_scenarios()
