# Build a new real map with wall follower 2.3 on Ubuntu

Use the bundle's Ubuntu runner from its top-level README. The runner configures the selected .103 session, domain 3 and CycloneDDS. Use a terminal on the Ubuntu desktop with a working RViz/OpenCV display. Use system Python 3 from ROS 2 Humble on Ubuntu 22.04.

## Dependencies

Alongside the navigation dependencies in the top-level README, the workstation needs the packages below. Run this command after configuring the ROS Humble apt repository.

```bash
sudo apt install build-essential cmake ros-humble-ament-cmake ros-humble-rclcpp \
  ros-humble-tf2 ros-humble-cartographer-ros ros-humble-nav2-map-server \
  ros-humble-rviz2 ros-humble-turtlebot3-description ros-humble-cv-bridge \
  ros-humble-tf2-geometry-msgs python3-opencv python3-numpy python3-yaml
```

## Run

1. Choose the **build** subcommand in the main README. This compiles the included C++ source for this Ubuntu machine. Rebuild after editing the C++ source. To adjust a following distance or speed, edit `mapping/local_variants/wall_follower_2_3/wall_follower_2_3.yaml` and restart the mapping session. The next session reads the YAML on startup.
2. Place .103 at the physical start facing the intended direction. Clear the route and use the **map** subcommand. It prepares bringup and the JPEG camera, starts Cartographer, RViz, the camera viewer, See Marker, the marker recorder and wall follower 2.3. Once sensor and command checks pass, it arms the existing Pi velocity guard and starts driving.
3. In RViz look for the occupancy map and colored cylinders in `map`. A marker is recorded only after an actual detection. If the camera window is closed, the detector still consumes the compressed stream. To open the picture by itself, use the **camera** subcommand while the camera publisher is running.
4. Press **Enter** in the mapping terminal to stop the robot and save the measurements. Wait for the terminal to report the map and number of landmarks. Press **Enter** again to close the displays. Use the dated run printed by the terminal, under `baseline/TurtleBot-Real-2026-11-11/runs/Real_103_YYYY-MM-DD_HHMMSS/`. The `maps/` folder contains the occupancy YAML and PGM image, marker TXT, `landmarks.csv`, and the map-frame starting pose in `*.start_map.txt` if the recorder obtained it. In the main navigation instructions, select the new run explicitly to use it instead of the supplied exam map.

## What the Ubuntu adapter changes

- CMake uses the Linux compiler and system ROS libraries. The local 2.3 C++, tuning YAML, teammate detector, real camera/recorder adapters and mapping RViz configuration come from the dated handoff under `mapping/`. The camera uses `/turtlebot_test/camera/compressed` and the detector keeps the fixed pullback at 0.00 m.
- The launcher runs the two Cartographer nodes using the course's original `turtlebot3_lds_2d.lua`, a 0.05 m occupancy grid, and wall time. It opens the package RViz configuration in Ubuntu's `rviz2`. The two-node launch file avoids a second RViz instance on systems where the stock TurtleBot3 launcher starts RViz unconditionally.
- The preflight uses Linux `/proc` to identify the exact compiled follower. It retains the source's sensor freshness, single-publisher, odometry, TF and QoS checks. The mapping and navigation entry points share the bundle's Pi controller. If the robot stays still, inspect `logs/preflight.log`, `mapping_preflight.json` and the Pi guard status.
- Building and previewing the camera are independent of starting the wall follower. The control flow and map-saving sequence remain visible in `manual_mapping.py`.

The hardware baseline was recorded on a Mac. The Ubuntu adapters were checked offline. Repeating the route on the teammate's Ubuntu machine is the live validation step.
