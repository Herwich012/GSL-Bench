# GSL-Bench
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04LTS-brightgreen.svg)
![IsaacSim 2022.2.0](https://img.shields.io/badge/IsaacSim-2022.2.0-brightgreen.svg)
### High Fidelity Gas Source Localization Benchmarking Suite


**GSL-Bench** is a benchmarking suite for gas source localization algorithms built with the [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) framework which uses [NVIDIA
Omniverse](https://docs.omniverse.nvidia.com/) and [Isaac
Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html). It is designed to provide an easy yet powerful way to simulate and evaluate gas source localization tasks. 

GSL-Bench extends the Pegasus Simulator framework by introducing GSL environments and sensors. Included are six GSL environments, more environments may be created with [AutoGDM+](https://github.com/tudelft/autoGDMplus). Also included are modules that automatically test the performance of algorithms and plot metrics. At the moment, only multirotor vehicles are supported, with support for other vehicle topologies planned for future versions. Check out [this video](https://youtu.be/kZa48WXf_1w?si=rkANmCbrxB9xOoii) to get an impression of GSL-Bench.


## Installation
ℹ️ GSL-Bench has only been verified to work with `Ubuntu 20.04 LTS` and `Isaac Sim 2022.2.0`

Follow the installation instructions of the pegasus simulator framework provided by their documentation [here](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html). 

⚠️ Note, instead of cloning the original PegasusSimulator repository as described by their documentation, clone this fork and checkout the `gsl-benchmarking` branch as such: 

#### Option 1: With HTTPS
```
git clone https://github.com/Herwich012/PegasusSimulator.git
git checkout gsl-benchmarking 
```
#### Option 2: With SSH (you need to setup a github account with ssh keys)
```
git clone git@github.com:Herwich012/PegasusSimulator.git
git checkout gsl-benchmarking 
```

## Usage

### Running the simulation
GSL-Bench uses Isaac Sim's Python API in [standalone mode](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html). To run GSL-Bench, open a terminal, `cd` into the `PegasusSimulator` directory and start Isaac Sim with a python script of choice given in the `examples` directory:

```
cd PegasusSimulator
ISAACSIM_PYTHON examples/10_python_single_vehicle_gsl.py
```

This python script specifies the following parameters of the simulation:
- Selected GSL algorithm
- Selected environment
- Starting postition
- Experiment ID
- Amount of runs before closing the simulation
- Logging of statistics
- Sensor settings
- Multitoror controller settings
- Stop condition(s)

### Creating a GSL algorithm
To create a new GSL algorithm, create a new Python file in:
`extensions/pegasus.simulator/pegasus/simulator/logic/gsl/`

It should contain your GSL algorithm class that inherits the `GSL` class from `gsl.py`. Add the appropriate import statement in the `__init__.py` contained in the same folder. Your GSL algorithm requires at least `get_wp()` function should return a 3x3 numpy array with the desired XYZ location, velocity and acceleration at the destination waypoint:

```
[[px, py, pz],
 [vx, vy, vz],
 [ax, ay, zx]]
```
It is also advised to include a `reset()` function which is called when the simulation resets in between runs. Please see the included algorithms, they can serve as an example. 

The GSL algorithm is used in a `nonlinear_controller` Python file which are situated in:
`examples/utils/`. There, the GSL algorithm is initialized and the `get_wp()` function is called when required.

### Performing a benchmark
To automatically perform multiple simulations with different algorithms accross multiple environments, a python script `benchmark.py` is provided in the root of the repository. By providing the Benchmark class with a dictionary of the required parameters and executing it through the terminal:

```
cd PegasusSimulator
python3 benchmark.py
```

It will do the following
1) It creates a list of experiments to be carried out: it runs every script(algorithm) in every environment from every position.
2) Each experiment is given an ID (by default, starting from the first available ID based on the results already present in `examples/results/`, or starting from a specified `start_id`). And a promt is shown to confirm the list of experiments:

```
Benchmarking: Ecoli3D
The following experiments are in queue:
 exp_id                      script                         env_id   start_pos
--------------------------------------------------------------------------------
['262', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [3.0, 3.0, 0.2]]
['263', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [7.5, 3.0, 0.2]]
['264', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [12.0, 3.0, 0.2]]
['265', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [3.0, 7.5, 0.2]]
['266', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [7.5, 7.5, 0.2]]
['267', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [12.0, 7.5, 0.2]]
['268', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [3.0, 12.0, 0.2]]
['269', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [7.5, 12.0, 0.2]]
['270', 'examples/12_python_single_vehicle_gsl_benchmark.py', 1, [12.0, 12.0, 0.2]]
['271', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [3.0, 3.0, 0.2]]
['272', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [7.5, 3.0, 0.2]]
['273', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [12.0, 3.0, 0.2]]
['274', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [3.0, 7.5, 0.2]]
['275', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [7.5, 7.5, 0.2]]
['276', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [12.0, 7.5, 0.2]]
['277', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [3.0, 12.0, 0.2]]
['278', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [7.5, 12.0, 0.2]]
['279', 'examples/12_python_single_vehicle_gsl_benchmark.py', 2, [12.0, 12.0, 0.2]]
Continue? (y/n):
```

3) After continuing by entering `y`, it sequentially performs every experiment.
4) A `.txt` file with the list of experiments is saved in the root folder.

### Plotting the results
To plot the logged statistics, there are scripts available in `examples/utils/plot/`. Specify the desired experiment ID(s) for plotting and run the script. The plots are saved in `examples/utils/plot/figures` by default.

## Citations

If you find GSL-Bench useful in your academic work, please cite the paper below. It is also available [here]().

```
@misc{
}
```

If you find Pegasus Simulator useful in your academic work, please cite the paper below. It is also available [here](https://arxiv.org/abs/2307.05263).
```
@misc{jacinto2023pegasus,
      title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
      author={Marcelo Jacinto and João Pinto and Jay Patrikar and John Keller and Rita Cunha and Sebastian Scherer and António Pascoal},
      year={2023},
      eprint={2307.05263},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Support and Contributing

## Licenses
Pegasus Simulator is released under [BSD-3 License](LICENSE). The license files of its dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

NVIDIA Isaac Sim is available freely under [individual license](https://www.nvidia.com/en-us/omniverse/download/).

PX4-Autopilot is available as an open-source project under [BSD-3 License](https://github.com/PX4/PX4-Autopilot).


