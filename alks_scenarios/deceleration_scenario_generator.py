"""
Create deceleration scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, p. 56
"""

from __future__ import annotations

import json
import numpy as np

from loguru import logger
from pathlib import Path
from tqdm import tqdm

from simple_scenario import Scenario, EgoConfiguration, Vehicle
from simple_scenario.road import SyntheticRoad, StraightSegment


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
        create_text: bool = False,
        create_openlabel: bool = False,
    ) -> None:
        # Each scenario will get its own subfolder under the result directory

        n_scenarios = len(self._gx_values) * len(self._ve0_kmh_values)

        with tqdm(total=n_scenarios) as progress_bar:
            for gx in self._gx_values:
                for ve0_kmh in self._ve0_kmh_values:
                    self._create_single_scenario(
                        gx,
                        ve0_kmh,
                        create_openx=create_openx,
                        create_image=create_image,
                        create_gif=create_gif,
                        create_text=create_text,
                        create_openlabel=create_openlabel,
                    )
                    progress_bar.update(1)

    def _create_single_scenario(
        self,
        gx: float,
        ve0_kmh: float,
        create_openx: bool = False,
        create_gif: bool = False,
        create_image: bool = False,
        create_text: bool = False,
        create_openlabel: bool = False,
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
        object_vehicle_length = Vehicle(0).length
        dx = ve0 * self._thw0
        object_s0 = (
            self._ego_s0 + dx + ego_configuration.length / 2 + object_vehicle_length / 2
        )
        object_a0 = -gx * self._g

        object_vehicle = Vehicle(
            0,
            start_lanelet_id=self._object_lanelet_id,
            start_s=object_s0,
            start_t=self._ego_t0,
            v0=vo0,
            a0=object_a0,
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
        ego_configuration = EgoConfiguration(
            self._ego_lanelet_id,
            self._ego_s0,
            self._ego_t0,
            ve0,
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

        scenario_dir = self._result_dir / scenario_name
        scenario_dir.mkdir(exist_ok=True)

        scenario_config_dir = scenario_dir / "configs"
        scenario_config_dir.mkdir(exist_ok=True)
        scenario.save(scenario_config_dir)

        scenario_openx_dir = scenario_dir / "openx"
        if create_openx or create_openlabel or create_text:
            scenario_openx_dir.mkdir(exist_ok=True)
        if create_openx:
            scenario.save(scenario_openx_dir, mode="openx")
        if create_text:
            _create_info_text(scenario_openx_dir, scenario_name, ve0_kmh, gx)
        if create_openlabel:
            _create_openlabel(
                scenario_openx_dir,
                scenario_name,
                ve0_kmh,
                self._n_lanes,
                self._lane_width,
                round(gx * self._g, 3),
            )
        if create_image or create_gif:
            scenario_image_dir = scenario_dir / "images"
            scenario_image_dir.mkdir(exist_ok=True)
        if create_image:
            scenario.render(scenario_image_dir, dpi=600)
        if create_gif:
            scenario.render_gif(scenario_image_dir, dpi=600)


def _create_info_text(
    scenario_openx_dir: Path,
    scenario_name: str,
    ve0_kmh: int,
    gx: float,
) -> None:
    info_text_path = scenario_openx_dir / f"{scenario_name}.txt"
    with Path.open(info_text_path, "w") as f:
        f.write(
            "This is a scenario based on the UN/ECE R.157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4 "
            "for the Deceleration defined in Annex 3. The parameters are given here: ve0: initial "
            "speed of the ego vehicle, gx: deceleration of the object vehicle as a fraction of g.\n"
        )
        f.write(f"Scenario Name: {scenario_name}\n")
        f.write(f"ve0 (km/h): {ve0_kmh}\n")
        f.write(f"gx (fraction of g): {gx:.2f}\n")
        f.write(f"Deceleration (m/s\u00b2): {gx * 9.81:.3f}\n")
    logger.info(f"Created info text at {info_text_path}")


def _create_openlabel(
    scenario_openx_dir: Path,
    scenario_name: str,
    ve0_kmh: int,
    lane_count: int,
    lane_width: float,
    deceleration_ms2: float,
) -> None:
    openlabel = {
        "openlabel": {
            "metadata": {
                "schema_version": "1.0.0",
                "tagged_file": f"{scenario_name}.xosc",
                "name": scenario_name,
                "ScenarioId": scenario_name,
                "description": (
                    "Deceleration scenario from UN/ECE R.157 "
                    "E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3"
                ),
                "annotator": "ika RWTH-Aachen",
                "comment": f"ve0 (km/h): {ve0_kmh}, deceleration (m/s²): {deceleration_ms2:.3f}",
                "ScenarioDatabase": "ALKS - Simple Scenario",
                "OpenXAvailability": {"Osc": True, "Odr": True, "Osg": False},
            },
            "ontologies": {
                "0": {"uri": "https://openlabel.asam.net/V1-0-0/ontologies/"}
            },
            "tags": {
                # Object vehicle decelerates; value is rate of deceleration in m/s²
                "0": {
                    "type": "MotionDecelerate",
                    "ontology_uid": "0",
                    "motionDecelerateValue": deceleration_ms2,
                },
                # Ego vehicle drives straight throughout
                "1": {"type": "MotionDrive", "ontology_uid": "0", "motionDriveValue": ve0_kmh},
                # Road type: 2-lane motorway without active traffic management
                "2": {"type": "MotorwayUnmanaged", "ontology_uid": "0"},
                # Road geometry: straight (StraightSegment only, no curves)
                "3": {"type": "HorizontalStraights", "ontology_uid": "0"},
                # Road geometry: no gradient defined
                "4": {"type": "LongitudinalLevelPlane", "ontology_uid": "0"},
                # Road cross-section: lanes run in the same direction
                "5": {"type": "TransverseDivided", "ontology_uid": "0"},
                # Lane type: standard traffic lanes
                "6": {"type": "LaneTypeTraffic", "ontology_uid": "0"},
                # Both ego and object are passenger cars ("medium" vehicle type)
                "7": {"type": "VehicleCar", "ontology_uid": "0"},
                # Subject (ego) vehicle speed in km/h
                "8": {
                    "type": "SubjectVehicleSpeed",
                    "ontology_uid": "0",
                    "subjectVehicleSpeedValue": ve0_kmh,
                },
                # Number of lanes on the road
                "9": {
                    "type": "LaneSpecificationLaneCount",
                    "ontology_uid": "0",
                    "laneSpecificationLaneCountValue": lane_count,
                },
                # Lane width in metres
                "10": {
                    "type": "LaneSpecificationDimensions",
                    "ontology_uid": "0",
                    "laneSpecificationDimensionsValue": lane_width,
                },
            },
        }
    }
    openlabel_path = scenario_openx_dir / f"{scenario_name}.json"
    with Path.open(openlabel_path, "w") as f:
        json.dump(openlabel, f, indent=2)
    logger.info(f"Created OpenLABEL file at {openlabel_path}")


def generate_all_scenarios() -> None:
    result_dir = Path(__file__).parent / ".." / "results" / "annex3" / "deceleration"
    result_dir.mkdir(exist_ok=True, parents=True)

    scenario_generator = DecelerationScenarioGenerator(result_dir)
    scenario_generator.create_all_scenarios(
        create_openx=True,
        create_image=True,
        create_gif=False,
        create_text=True,
        create_openlabel=True,
    )


if __name__ == "__main__":
    generate_all_scenarios()
