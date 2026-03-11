from pathlib import Path

from alks_scenarios import CutOutScenarioGenerator


class TestScenarioGeneration:
    RESULT_DIR = Path(__file__).parent / "test_results" / "test_scenario_generation"
    RESULT_DIR.mkdir(exist_ok=True, parents=True)

    def test_cutout_scenario_generator(self):
        cutout_generator = CutOutScenarioGenerator()

        # Check whether general scenario generation works
        generated_scenario = cutout_generator.get_single_scenario(
            dx0_f=70, vy=0.5, ve0=60 / 3.6
        )
        assert generated_scenario is not None

        generated_scenario_high_speed = cutout_generator.get_single_scenario(
            dx0_f=100, vy=0.75, ve0=130 / 3.6
        )
        assert generated_scenario_high_speed is not None

        # Check whether feasiblity check works
        assert (
            cutout_generator.get_single_scenario(
                dx0_f=10, vy=0.1, ve0=130 / 3.6, check_collision_feasibility=False
            )
            is not None
        )
        assert (
            cutout_generator.get_single_scenario(
                dx0_f=10, vy=0.1, ve0=130 / 3.6, check_collision_feasibility=True
            )
            is None
        )

        print("All tests passed.")


if __name__ == "__main__":
    tester = TestScenarioGeneration()
    tester.test_cutout_scenario_generator()
