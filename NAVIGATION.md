# Save markers and navigate them (ROS 2 Humble)

The first run still uses `wall_follower.launch.py`, which starts `see_marker.py` and `point_transformer.py`. Keep TurtleBot3 Cartographer running as in `README`.

```bash
# With TurtleBot3 simulator and Cartographer already running:
ros2 launch wall_follower wall_follower.launch.py use_sim_time:=true
# After the follower returns, while Cartographer is still running, save the map:
ros2 run nav2_map_server map_saver_cli -f "$HOME/maze_map"
```

When the wall follower returns to its start, `point_transformer.py` saves `~/markers.txt`: the odom start x,y,heading in radians, then four map-frame x,y,marker-type rows. That is the file the assignment specifies, and it is written first so nothing optional can stop it from reaching disk. It still saves the original `landmarks.csv`. If you interrupt before it finishes, Ctrl-C also writes the files. Check that it reports **four** markers: navigation rejects incomplete files.

It then tries to write `~/markers.start_map.txt`, holding the same start pose expressed in the map frame. This one is an extra, because the marker rows are in the map frame while the required start row is in odom, and those two frames only share an origin if Cartographer and odometry were started together. If the `map` -> `odom` transform never arrived, the recorder logs a warning, deletes any sidecar left from an earlier run, and navigation reads the odom start row as a map pose instead.

After stopping Cartographer and the wall-follower launch, place the robot at the same starting position **and heading** and run:

```bash
ros2 launch wall_follower navigate_markers.launch.py map:="$HOME/maze_map.yaml" use_sim_time:=true
```

Use `use_sim_time:=false` on a real robot. The launch needs `TURTLEBOT3_MODEL` exported, because `turtlebot3_navigation2`'s launch file reads it at import time and raises `KeyError` before anything starts.

The navigation launch starts the TurtleBot3 Nav2 map server and AMCL, sets AMCL's initial pose from `markers.start_map.txt` when that file is present and from the `markers.txt` start row when it is not, and visits the four marker rows in **file order**. The node logs which of the two it used. You may reorder those four rows without touching the first line or the sidecar.

Nav2 plans to a reachable point 0.5 m from each marker, not the occupied cylinder centre. Sixteen candidate points around the marker are tested with `ComputePathToPose`, and a candidate is accepted only if the returned path actually ends at it: NavFn runs with `tolerance: 0.5` in the TurtleBot3 params and otherwise reports success for a path that stops at the wall in front of an unreachable point. If no approach point can be planned or a goal fails, the node stops instead of skipping a marker.

To choose a different output/input path, pass `marker_file:=/absolute/path/markers.txt` to both launches (the recording launch forwards it to `point_transformer.py`); the map-start sidecar is stored alongside it. Map and markers must be from the **same** run. Build with `colcon build --packages-select wall_follower` in the TurtleBot3 ROS 2 Humble workspace, then source `install/setup.bash`.
