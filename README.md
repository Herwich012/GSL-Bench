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
1) Clone this fork of the Pegasus Simulator repository and `checkout` the `gsl-benchmarking` branch.
2) Follow the installation instructions of the pegasus simulator framework provided by their documentation [here](https://pegasussimulator.github.io/PegasusSimulator/).


## Usage

### Creating a GSL algorithm
To create an GSL algorithm, create a new Python module with the algorithm class that inherits the `GSL` class from `gsl.py`. 

### Running the simulation

### Performing a benchmark

### Plotting the results


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


