# TurtleBot .103 | 11 November 2026 | Ubuntu reproduction guide

**Use `./ubuntu/run.sh navigate` to repeat the successful three-stop exam. Use `./ubuntu/run.sh map` only when you need a new map with Wall Follower 2.3.** Each command owns the wheels for its own stage. Navigation loads saved coordinates and uses Nav2; it does not run a wall follower or search for colors while driving. The working language of this guide and the entry commands is English. Some original progress messages are in Chinese.

| What is in the ZIP | Origin and evidence |
|---|---|
| Manual exam navigation | On 2026-11-11 at 16:59:16 the Mac and physical TurtleBot **192.168.1.103** ran yellow/pink → blue/pink → pink/green → original mapping start. At 17:03:14 the report recorded all three stops, `home_reached: true` and `zero_command_confirmed`. |
| Ready-to-use map | The **15:27** physical run produced the occupancy PGM + YAML, three measured markers, CSV and map-frame starting pose. The mapping controller for that run was **wall_follower_3**, not 2.3. |
| Optional new mapping | The team's **2.3** controller, hardware See Marker (revision `aa52fb5294769d0cfefede48f022e21ec98b90ff`), compressed camera, Cartographer, RViz, recorder and Linux build instructions are included. This creates a **new** dated run. No completed 2.3 physical mapping run is claimed. |

## 1. On the Ubuntu laptop: unpack and preview

Target: **Ubuntu 22.04 (Jammy), ROS 2 Humble**, system Python 3.10, a graphical desktop. This release targets the already configured course Waffle Pi .103. Place the ZIP in Downloads. Keep the extracted directory writable and keep its subdirectories together. Example:

```bash
cd ~/Downloads
unzip TurtleBot_2026-11-11_Ubuntu.zip
cd TurtleBot_2026-11-11_Ubuntu
sha256sum --quiet -c SHA256SUMS
sudo apt install python3-yaml python3-pil     # only if not installed
./ubuntu/run.sh preview
```

Preview reads only local files; no ROS installation, Wi-Fi or robot is needed. Its default route is **start → yellow/pink → blue/pink → pink/green → home**. The first color is the upper half. This particular map has no fourth measured marker. Keep a clean copy of the ZIP if you plan to edit or generate files.

For driving, first install [ROS 2 Humble Desktop for Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), including its apt repository. In a fresh desktop terminal, install the host packages:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-rmw-cyclonedds-cpp \
  ros-humble-tf2-geometry-msgs ros-humble-turtlebot3-description \
  python3-pil python3-yaml python3-numpy openssh-client iproute2
./ubuntu/run.sh check nav
```

The launcher sources `/opt/ros/humble/setup.bash` itself. You do not need Gazebo, the macOS course environment, a second RViz, or the original Mac's IP address. If your Ubuntu is 24.04 with Jazzy, this exact stack was not validated there; use a Jammy/Humble desktop for this reproduction.

## 2. Pair once, then repeat the exam

Join the same robot LAN, make sure this is your turn to use .103, and place the robot on the floor at the original physical start with the maze and landmarks in their mapped positions. If someone was using a different Pi session, have them finish it; an old hardware driver that still owns the robot must exit. The [Pi pairing guide](ubuntu/ROBOT-NETWORK.md) describes switching computers. Do not launch a second copy of bringup by hand.

```bash
./ubuntu/run.sh setup       # first use on this Ubuntu laptop; dedicated pairing, no motion
./ubuntu/run.sh status      # inspect the paired Pi, may show no services after boot
./ubuntu/run.sh navigate    # manual fallback; loads the bundled 15:27 map by default
```

Setup verifies the specific Pi's host key and Wi-Fi MAC, and installs the archived guard/JPEG helper and a dedicated unicast DDS configuration in its own Pi directory. Enter the Pi password when SSH asks; a password is not in the package. The workstation and Pi use real time, domain 3, CycloneDDS, and the laptop's actual route to .103. Navigation starts missing services and requires a disarmed, healthy robot before driving. If the laptop IP changes, follow the pairing guide and repeat setup.

1. Wait for **Physical maze** in RViz. Select **2D Pose Estimate**, press at the robot's *actual position* and drag along its heading. AMCL confirms localization; Nav2 then activates the costmaps. A colored cylinder alone is not evidence that the map or localization is ready.
2. Wait for the terminal's prompt to click **2D Goal Pose** (`可以点击`; the mission log records `[RVIZ_TRIGGER_READY]`). Then select that tool and drag **one** arrow on the map. The arrow is a start button: its coordinates do not become a destination. The saved CSV supplies all goals. An earlier click is deliberately ignored.
3. Nav2 visits the measured landmarks in order, then sends the saved **mapping start** as its final goal. The straight-line target/limit is **0.13 m/s**; turns and approaches can be slower. The nominal approach distance is **0.40 m** from a saved marker coordinate; the baseline measured 0.46, 0.48 and 0.48 m on arrival.
4. **Stop:** press **Enter in the launching terminal** to request a stop. If that terminal is unavailable, use `./ubuntu/run.sh stop` from a second terminal in the bundle. A zero-speed acknowledgement is printed on successful disarm. Each navigation creates its own timestamped log folder under the selected run; check `navigation-result.json` for the outcome, `home_reached` and `stop_ack`.

## 3. Change the teacher's visit order in English

Copy the example, edit the rows, preview, then issue the same command with `navigate`. This changes visit order only. Each color pair measured on this map must occur **exactly once**. The map coordinates remain tied to the color in the measurement file.

```bash
cp examples/teacher-order.csv my-order.csv
nano my-order.csv
./ubuntu/run.sh preview  --order-csv "$PWD/my-order.csv" --expect-count 3
./ubuntu/run.sh navigate --order-csv "$PWD/my-order.csv" --expect-count 3
```

Example: `marker` on the first line, followed by `pink/green`, `yellow/pink`, `blue/pink` on separate lines. The manual exam handles **1–4 distinct measured pairs** and returns home. It rejects missing, repeated or unmeasured pairs. Adding a new row does not measure a marker. Remap if the physical geometry or landmark positions change. Each live attempt freezes its CSV as `teacher-order.csv` in its own navigation log folder.

## 4. Optional: create a new map with Wall Follower 2.3

Install these additional packages; then build on the Ubuntu laptop. See [mapping notes](ubuntu/mapping-tools/README.md) for the Linux adapters and tuning. The original 2.3 C++ is also included under `mapping/reference/`.

```bash
sudo apt install build-essential cmake ros-humble-ament-cmake ros-humble-rclcpp \
  ros-humble-tf2 ros-humble-cartographer-ros ros-humble-cv-bridge python3-opencv
./ubuntu/run.sh check mapping
./ubuntu/run.sh build
# After setup from step 2, place the robot at the desired start and clear the route.
./ubuntu/run.sh map
```

**Mapping can start driving as soon as its checks pass.** The command starts the base, JPEG camera, Cartographer, 2.3, RViz, See Marker and recorder. It opens a camera window and reports new detections with map coordinates. This stage's Pi guard limits forward speed to **0.06 m/s**. The fixed camera pullback is **0.00 m**. The `camera` subcommand can open another viewer while the compressed stream already exists.

**Enter once to stop and save**, wait for the completion message, then **Enter again to close mapping RViz** and release the run lock. Copy the new run's name from the terminal (e.g. `Real_103_YYYY-MM-DD_HHMMSS`; replace the placeholder below). Old records are kept. The fresh run must include `maps/maze_*.yaml`, its matching `.pgm`, `maps/markers_*.txt`, the `.start_map.txt` sidecar and `maps/landmarks.csv` to do the manual exam.

```bash
RUN=Real_103_YYYY-MM-DD_HHMMSS   # replace with the printed name
./ubuntu/run.sh preview "$RUN"
./ubuntu/run.sh navigate "$RUN"
```

Omitting RUN deliberately selects the **shipped 15:27 baseline**, even after you remap. To change order on a new map, pass that same RUN and an order CSV for that map to both commands. For 2.3 tuning, edit `mapping/local_variants/wall_follower_2_3/wall_follower_2_3.yaml` and restart mapping; C++ changes require `./ubuntu/run.sh build` again.

## Evidence and troubleshooting for a teammate or coding agent

- **Source of truth:** `baseline/TurtleBot-Real-2026-11-11/runs/Real_103_2026-11-11_152732/maps/` contains the saved map and markers. Its sibling `导航_Real_2026-11-11_165916/` contains the original `navigation-result.json`, CSV snapshot, localization, preflight, Nav2 and mission logs. Absolute Mac paths inside old logs are historical.
- **Implementation:** `ubuntu/navigation.py` selects the frozen `--recorded --exam --rviz-start` path and supplies Linux RViz/Pi locations. Saved mission logic is in `baseline/TurtleBot-Real-2026-11-11/scripts/{manual_navigation,navigate_saved_markers,exam_marker_order}.py`; Nav2 tuning is in `config/nav2_robot103.yaml` in that project. The frozen automatic-start prototype is not an Ubuntu entry point in this handoff. `mapping/PROVENANCE.json` identifies controller source, commit and sample provenance. `archives/` contains the two unchanged original ZIPs. Do not launch the old macOS `.command` files on Linux.
- **RViz shows only markers:** wait for the map and then initialize **2D Pose Estimate**. Occupancy is `/map` (Reliable / Transient Local); inflation is published on the global/local costmaps after Nav2 activation, with an inflation radius of 0.40 m in this config. If no map or no TF arrives, inspect this run's `nav2.log`, `localization.log`, `rviz.log` and `checks.json`. The navigation preflight compares the live grid contents and selected map path; do not fix this by inventing a static map transform.
- **SSH succeeds but the robot does not move:** check `./ubuntu/run.sh status`, Pi/laptop clock agreement, and whether live scan/odom and the guarded velocity path are present. Commands for inspecting the same DDS session are in [ROBOT-NETWORK.md](ubuntu/ROBOT-NETWORK.md). A mismatched SSH identity, foreign guard, double bringup or different DDS URI means stop and identify the active robot/session; do not change the pin or kill another operator's nodes.
- **Validation boundary:** the three stops and return were measured with a Mac controlling the physical robot. In a disconnected Ubuntu 22.04.5 / ROS 2 Humble Nav2 1.1.20 container, the same 2.3 C++ compiled, the seven route checks passed, the Nav2 lifecycle groups activated, and all **7,656** cells of the live 88 × 87 grid matched the bundled PGM. The container used Fast DDS and had no Cartographer ROS or CycloneDDS installation; the real-robot commands require both for the full two-stage workflow. The Ubuntu camera adapter decoded a synthetic JPEG; that does not verify physical optics or the teammate's Wi-Fi. Ubuntu-to-.103 driving and a complete new 2.3 maze run still require field verification.
