# Dockerized BOSCH Future Mobility Challenge Simulator

Docker-based setup for running the BFMC autonomous vehicle simulator with ROS Noetic and Gazebo 11.

> Developed at the [SenseLAB](http://senselab.tuc.gr/) of the [Technical University of Crete](https://www.tuc.gr/el/archi)

[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python 3](https://img.shields.io/badge/python-3.x-blue.svg)](https://www.python.org/downloads/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](http://gazebosim.org/)
---

## Overview

This repository provides a complete Docker environment for the BOSCH Future Mobility Challenge (BFMC) simulator, enabling easy development and testing of autonomous vehicle algorithms without complex local installation.

For the original simulator source code and documentation, visit: https://github.com/ECC-BFMC/Simulator

## Quick Start

```bash
# Build Docker image (ensure the tag matches the run scripts below)
docker build -t bfmc_sim .

# Run container (local desktop X11 session)
./run_bfmc.sh

# Or, if you are connecting remotely (e.g. via xrdp) use the remote helper
./run_bfmc_rmt.sh         # launches with DISPLAY from the remote session
./run_bfmc_rmt.sh --headless   # launch without GUI
./run_bfmc_rmt.sh --no-gpu     # force software rendering

# Inside container - build and launch
cd /root/Documents/Simulator
catkin_make --pkg utils
catkin_make
source devel/setup.bash
roslaunch sim_pkg map_with_all_objects_REC.launch
```

## Troubleshooting

- **Display not found / ERROR: DISPLAY not set**: If you see an error indicating that `DISPLAY` is not set or that the display cannot be opened, try one of the following:
	- If running locally, ensure you have an X server and that `DISPLAY` is set (for many desktops it is `:0`). You can check with `echo $DISPLAY` and, if needed, export it manually before running `./run_bfmc.sh`:

		```bash
        echo $DISPLAY
		export DISPLAY=$DISPLAY
		./run_bfmc.sh
		```

	- If connecting remotely via RDP or another remote desktop, use `./run_bfmc_rmt.sh` which picks up the remote session `DISPLAY`. If `DISPLAY` is still empty in that session, set it to the value your remote desktop provides (for xrdp sessions this is usually available in the session environment).

- **Docker image tag mismatch**: When building the Docker image you specify a tag with `-t`. Both `./run_bfmc.sh` and `./run_bfmc_rmt.sh` currently expect the image to be named `bfmc_sim`. If you build with a different tag, either rebuild with the matching tag or edit the run script(s) to use the tag you built. Example:

		```bash
		# Build with a custom tag
		docker build -t my_bfmc_image .

		# Edit run scripts or run directly
		# In run_bfmc.sh replace the final image name @line 80  
        # In run_bfmc_rmt.sh replace the final image name @line 137
        # or run directly:
		docker run ... my_bfmc_image
		```

- **GPU / rendering issues**: If Gazebo shows rendering errors, try forcing software rendering by adding `--no-gpu` to `./run_bfmc_rmt.sh` or setting `LIBGL_ALWAYS_SOFTWARE=1` in the environment. See the run scripts for GPU detection and flags.


## Features

- ROS Noetic Desktop Full
- Gazebo 11 with custom BFMC models
- NVIDIA GPU acceleration support
- Pre-configured environment with all dependencies
- Resource-optimized for stable simulation



## Requirements

- Docker
- 8GB+ RAM recommended
- NVIDIA GPU with Container Toolkit (optional, for better performance)

## Credits

Original simulator developed by the ECC-BFMC team: https://github.com/ECC-BFMC/Simulator
