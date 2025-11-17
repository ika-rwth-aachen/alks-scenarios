# alks-scenarios

<img src="https://github.com/ika-rwth-aachen/alks-scenarios/blob/main/alks-scenarios.svg?raw=True" width="400px" style="margin: 10px;">

Recreate test scenarios from [UN/ECE R.157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4](https://unece.org/transport/documents/2023/03/standards/un-regulation-no-157-amend4) Annex 3 pp. 45-56 using [simple-scenario](https://github.com/ika-rwth-aachen/simple-scenario).

# Examples

:bulb: *To recreate these gifs use: [examples/generate_gifs.py](https://github.com/ika-rwth-aachen/alks-scenarios/blob/main/demo/examples/generate_gifs.py).*

*Cut-in* test scenario:

<img src="https://raw.githubusercontent.com/ika-rwth-aachen/alks-scenarios/refs/heads/main/assets/cutin_plot_09_ve0_40_dv0_10_dx0_3.0_vy_0.3.gif">

*Cut-out* test scenario:

<img src="https://raw.githubusercontent.com/ika-rwth-aachen/alks-scenarios/refs/heads/main/assets/cutout_plot_04_ve0_30_dx0f_64.0_vy_0.5.gif">

*Deceleration* test scenario:

<img src="https://raw.githubusercontent.com/ika-rwth-aachen/alks-scenarios/refs/heads/main/assets/deceleration_plot_ve0_48_gx_0.50.gif">

# Notice

> [!IMPORTANT]
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).
> We cover a wide variety of research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.
> If you would like to learn more about how we can support your automated driving or robotics efforts, feel free to reach out to us!
> :email: ***opensource@ika.rwth-aachen.de***

## Install

To generate `alks-scenarios`, you must first clone the repository.

```bash
$ git clone git@github.com:ika-rwth-aachen/alks-scenarios.git
$ cd alks-scenarios
```

It is recommended to use [uv](https://docs.astral.sh/uv/getting-started/installation/) for package management.
If you do not want to use `uv`, please consult the [Without uv](#without-uv) section.

## With uv

Install requirements with

```bash
$ uv sync
```

To run a script, use

```bash
$ uv run /path/to/script.py
```

or directly use the python interpreter from the `.venv` folder in e.g. VSCode.

To run the tests, install the dev requirements with

```bash
$ uv sync --dev
```

and run the tests

```bash
$ uv run pytest
```

## Without uv

Install the project editable

```bash
$ python -m pip install -e .
```

To run the tests, first install pytest

```bash
$ python -m pip install pytest
```

and run

```bash
$ pytest
```

## Use

Use the following commands to generate test scenarios of different logical scenarios:

- [`alks_scenarios/cutin_scenario_generator.py`](https://github.com/ika-rwth-aachen/alks-scenarios/blob/main/alks_scenarios/cutin_scenario_generator.py): Generate *Cut-in* test scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, pp. 45-52
- [`alks_scenarios/cutin_scenario_generator.py`](https://github.com/ika-rwth-aachen/alks-scenarios/blob/main/alks_scenarios/cutout_scenario_generator.py): Generate *Cut-out* test scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, pp. 53-55 
- [`alks_scenarios/cutin_scenario_generator.py`](https://github.com/ika-rwth-aachen/alks-scenarios/blob/main/alks_scenarios/deceleration_scenario_generator.py): Generate *Deceleration* test scenarios from UNECE R157 E/ECE/TRANS/505/Rev.3/Add.156/Amend.4, Annex 3, p. 56

Per default, test scenarios are generated in `results/annex3`.
Per test scenario, the following files are generated:
- Descriptive top view image for visualization purposes (e.g., `cutin_plot_01_ve0_60_dv0_0_dx0_0.0_vy_0.1.png`)
- A [simple-scenario](https://github.com/ika-rwth-aachen/simple-scenario) config for loading the scenario into a [simple-scenario](https://github.com/ika-rwth-aachen/simple-scenario) `Scenario` object (e.g., `cutin_plot_01_ve0_60_dv0_0_dx0_0.0_vy_0.1.json`)
- A set of OpenSCENARIO and OpenDRIVE file for a simulation tool like esmini (e.g., `cutin_plot_01_ve0_60_dv0_0_dx0_0.0_vy_0.1.xosc`, `cutin_plot_01_ve0_60_dv0_0_dx0_0.0_vy_0.1.xodr`)

# Acknowledgements

This package is developed as part of the [SYNERGIES project](https://synergies-ccam.eu).

<img src="assets/synergies.svg" style="width:2in" />
<!-- <img src="https://raw.githubusercontent.com/ika-rwth-aachen/alks-scenarios/refs/heads/main/assets/synergies.svg" style="width:2in" /> -->

Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.

<img src="assets/funded_by_eu.svg" style="width:4in" />
<!-- <img src="https://raw.githubusercontent.com/ika-rwth-aachen/alks-scenarios/refs/heads/main/assets/funded_by_eu.svg" style="width:4in" /> -->
