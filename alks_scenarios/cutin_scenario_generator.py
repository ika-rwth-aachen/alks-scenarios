"""
Create cutin scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, pp. 45-52
"""

from __future__ import annotations

import json
import multiprocessing as mp
import numpy as np

from loguru import logger
from pathlib import Path
from tqdm import tqdm

from simple_scenario import Scenario, EgoConfiguration, Vehicle
from simple_scenario.road import SyntheticRoad, StraightSegment


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
        create_text: bool = False,
        create_openlabel: bool = False,
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
                create_text=create_text,
                create_openlabel=create_openlabel,
            )

    def _create_scenarios_of_plot_no(
        self,
        plot_no: int,
        ve0_kmh: float,
        dv0_kmh: float,
        create_openx: bool = False,
        create_image: bool = False,
        create_gif: bool = False,
        create_text: bool = False,
        create_openlabel: bool = False,
    ) -> None:
        # Create folders
        plot_no_result_dir = self._result_dir / f"plot_{plot_no:02d}"
        plot_no_result_dir.mkdir(exist_ok=True)

        # Each scenario will get its own subfolder under the plot directory

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
                        plot_no_result_dir,
                        create_gif=create_gif,
                        create_image=create_image,
                        create_openx=create_openx,
                        create_text=create_text,
                        create_openlabel=create_openlabel,
                    )
                    progress_bar.update(1)

    def _create_single_scenario(
        self,
        plot_no: int,
        dx0: float,
        vy: float,
        ve0: float,
        vo0: float,
        plot_no_result_dir: Path,
        create_gif: bool = False,
        create_image: bool = False,
        create_openx: bool = False,
        create_text: bool = False,
        create_openlabel: bool = False,
    ) -> None:
        ve0_kmh = ve0 * 3.6
        vo0_kmh = vo0 * 3.6
        ve0_kmh = int(np.round(ve0_kmh))
        dv0_kmh = int(np.round(ve0_kmh - vo0_kmh))
        scenario_name = f"cutin_plot_{plot_no:02d}_ve0_{ve0_kmh}_dv0_{dv0_kmh}_dx0_{dx0:.1f}_vy_{vy:.1f}"

        # Create simple scenario object
        ego_configuration = EgoConfiguration(
            start_lanelet_id=self._ego_lanelet_id,
            start_s=self._ego_s0,
            start_t=self._ego_t0,
            v0=ve0,
        )

        # Calculate object_vehicle_t0 from dy0
        ego_width = ego_configuration.width
        dummy_vehicle = Vehicle(0)
        object_vehicle_width = dummy_vehicle.width
        object_vehicle_t0_from_ego_llt = self._ego_t0 - (
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
            start_lanelet_id=self._object_lanelet_id,
            start_s=object_vehicle_s0,
            start_t=object_vehicle_t0,
            v0=vo0,
            lc_direction=1,
            lc_type="vy",
            lc_vy=vy,
        )
        scenario_duration = np.ceil(object_vehicle_lc_duration) + 10

        # Calculate road length
        ego_dist = ve0 * scenario_duration
        road_length = max(ego_dist, self._min_road_length) + self._ego_s0 + 100
        goal_position = self._ego_s0 + 0.75 * ego_dist
        ego_configuration = EgoConfiguration(
            start_lanelet_id=self._ego_lanelet_id,
            start_s=self._ego_s0,
            start_t=self._ego_t0,
            v0=ve0,
            target_s=goal_position,
            target_t=0,
            target_lanelet_id=self._ego_lanelet_id,
        )
        road = SyntheticRoad(
            self._n_lanes,
            self._lane_width,
            segments=[StraightSegment(road_length)],
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

        scenario_dir = plot_no_result_dir / scenario_name
        scenario_dir.mkdir(exist_ok=True)

        scenario_config_dir = scenario_dir / "configs"
        scenario_config_dir.mkdir(exist_ok=True)
        scenario.save(scenario_config_dir)

        scenario_openx_dir = scenario_dir / "openx"

        if create_openx or create_openlabel or create_text:
            scenario_openx_dir.mkdir(exist_ok=True)
        if create_openx:
            scenario.save(scenario_openx_dir, mode="openx")
        if create_image or create_gif:
            scenario_image_dir = scenario_dir / "images"
            scenario_image_dir.mkdir(exist_ok=True)
        if create_image:
            scenario.render(scenario_image_dir, dpi=600)
        if create_gif:
            scenario.render_gif(scenario_image_dir, dpi=600)
        if create_text:
            _create_info_text(
                scenario_openx_dir, scenario_name, plot_no, ve0_kmh, dv0_kmh, dx0, vy
            )
        if create_openlabel:
            vo0_kmh = int(np.round(vo0 * 3.6))
            _create_openlabel(
                scenario_openx_dir,
                scenario_name,
                int(plot_no),
                int(ve0_kmh),
                int(dv0_kmh),
                float(dx0),
                float(vy),
                vo0_kmh,
                float(object_vehicle_lc_duration),
                float(self._ego_s0),
                float(self._ego_t0),
                int(self._ego_lanelet_id),
                float(object_vehicle_s0),
                float(object_vehicle_t0),
                int(self._object_lanelet_id),
                float(self._dy0),
            )


def _create_openlabel(
    scenario_openx_dir: Path,
    scenario_name: str,
    plot_no: int,
    ve0_kmh: int,
    dv0_kmh: int,
    dx0: float,
    vy: float,
    vo0_kmh: int,
    object_vehicle_lc_duration: float,
    ego_s0: float,
    ego_t0: float,
    ego_lanelet_id: int,
    object_vehicle_s0: float,
    object_vehicle_t0: float,
    object_lanelet_id: int,
    dy0: float,
) -> None:
    openlabel = {
        "openlabel": {
            "metadata": {
                "schema_version": "1.0.0",
                "name": scenario_name,
                "description": (
                    "Cut-in scenario from UN/ECE R.157 "
                    "E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3"
                ),
                "annotator": "alks-scenarios",
                "comment": f"Plot {plot_no}",
            },
            "objects": {
                "0": {
                    "name": "ego",
                    "type": "Ego",
                    "object_data": {
                        "text": [{"name": "vehicle_type", "val": "medium"}],
                        "num": [
                            {"name": "v0_kmh", "val": ve0_kmh},
                            {"name": "s0_m", "val": ego_s0},
                            {"name": "t0_m", "val": ego_t0},
                            {"name": "lanelet_id", "val": ego_lanelet_id},
                        ],
                    },
                },
                "1": {
                    "name": "object_vehicle",
                    "type": "Vehicle",
                    "object_data": {
                        "text": [{"name": "vehicle_type", "val": "medium"}],
                        "num": [
                            {"name": "v0_kmh", "val": vo0_kmh},
                            {"name": "dx0_m", "val": dx0},
                            {"name": "s0_m", "val": object_vehicle_s0},
                            {"name": "t0_m", "val": object_vehicle_t0},
                            {"name": "lanelet_id", "val": object_lanelet_id},
                        ],
                    },
                },
            },
            "actions": {
                "0": {
                    "name": "cut_in",
                    "type": "CutIn",
                    "action_data": {
                        "text": [
                            {"name": "direction", "val": "left"},
                            {"name": "lc_type", "val": "constant_vy"},
                            {
                                "name": "regulation_reference",
                                "val": "UN/ECE R.157 Annex 3 Table 2",
                            },
                        ],
                        "num": [
                            {"name": "vy_m_per_s", "val": vy},
                            {"name": "duration_s", "val": object_vehicle_lc_duration},
                            {"name": "dy0_m", "val": dy0},
                        ],
                    },
                }
            },
            "relations": {
                "0": {
                    "name": "CutInActor",
                    "type": "IsActorOf",
                    "elements": {
                        "objects": {"1": {}},
                        "actions": {"0": {}},
                    },
                }
            },
            "tags": {
                "0": {"name": "scenario_type", "type": "CutIn"},
                "1": {"name": "regulation_type", "type": "ALKS"},
            },
        }
    }
    openlabel_path = scenario_openx_dir / f"{scenario_name}.json"
    with open(openlabel_path, "w") as f:
        json.dump(openlabel, f, indent=2)
    logger.info(f"Created OpenLABEL file at {openlabel_path}")


def _create_info_text(
    scenario_openx_dir: Path,
    scenario_name: str,
    plot_no: int,
    ve0_kmh: int,
    dv0_kmh: int,
    dx0: float,
    vy: float,
) -> None:
    info_text_path = scenario_openx_dir / f"{scenario_name}.txt"
    with open(info_text_path, "w") as f:
        f.write(
            f"This is a scenario based on the UN/ECE R.157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4 for the Cut-in defined in Annex 3. The Parameters are given here: ve0: initial speed of the ego vehicle, dv0: difference in initial speed between ego and object vehicle, dx0: initial longitudinal distance between ego and object vehicle, vy: lateral speed of the object vehicle during the cut-in maneuver.\n"
        )
        f.write(f"Scenario Name: {scenario_name}\n")
        f.write(f"Plot No: {plot_no}\n")
        f.write(f"ve0 (km/h): {ve0_kmh}\n")
        f.write(f"dv0 (km/h): {dv0_kmh}\n")
        f.write(f"dx0 (m): {dx0:.1f}\n")
        f.write(f"vy (m/s): {vy:.1f}\n")
    logger.info(f"Created info text at {info_text_path}")


def unpack_and_run(args_list: list) -> None:
    scenario_generator = CutInScenarioGenerator(args_list[0])
    scenario_generator.create_all_scenarios(
        only_plot_no=args_list[1],
        create_image=args_list[2],
        create_openx=args_list[3],
        create_gif=args_list[4],
        create_text=args_list[5],
        create_openlabel=args_list[6],
    )


def generate_all_scenarios() -> None:
    result_dir = Path(__file__).parent / ".." / "results" / "annex3" / "cutin"
    result_dir.mkdir(exist_ok=True, parents=True)

    create_image = True
    create_openx = True
    create_gif = False
    create_text = True
    create_openlabel = True

    n_plots_in_regulation = 1  # Number of plots defined in the regulation

    n_workers = n_plots_in_regulation

    all_args_lists = [
        [
            result_dir,
            i + 1,
            create_image,
            create_openx,
            create_gif,
            create_text,
            create_openlabel,
        ]
        for i in range(n_plots_in_regulation)
    ]

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
