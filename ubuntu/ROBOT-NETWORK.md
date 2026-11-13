# Pair an Ubuntu laptop with TurtleBot .103

Use this guide from the extracted ZIP root. The course Pi already has ROS 2 Humble, its TurtleBot hardware workspace, LDS-01 and the physical camera. Connect the laptop to the robot LAN. Agree with the other operator who has control for this run. Two computers must not run navigation or mapping on the same robot at the same time.

## Reuse the original Pi session

```bash
bash ubuntu/robot.sh setup
bash ubuntu/robot.sh status
```

SSH asks for the Pi account password on first use if your own default SSH key is unavailable. The helper verifies the expected SSH host key and WLAN MAC of `192.168.1.103`. An identity mismatch requires verifying the hardware with the lab staff. Setup leaves all Pi files and processes intact. A password prompt may appear on the terminal even when the launcher logs its progress elsewhere.

The pairing stores the active laptop IPv4 address and a CycloneDDS XML under `~/.config/tb3-real103/`. Legacy mode uses domain 3 and the archived discovery tag. The laptop sends unicast discovery to .103. The original Pi XML lists the original Mac at `192.168.1.167` as a peer. Verify actual scan delivery on a different laptop before relying on this pairing.

If all service PID lists in the status output are empty and this is your turn, the navigation launcher can restore the hardware services. To inspect the scan before navigation, run the following. `prepare` starts missing drivers and the guard in a disarmed state.

```bash
bash ubuntu/robot.sh prepare
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$(bash ubuntu/robot.sh env)"
timeout 15 ros2 topic echo /scan sensor_msgs/msg/LaserScan --qos-reliability best_effort --field header --once
```

A received header verifies that fresh sensor traffic reaches this laptop. Before enabling the wheels the archived Pi program separately verifies the ROS endpoints and recent velocity input on the Pi. Follow the main README to launch the mission. The launcher handles arming. Use `bash ubuntu/robot.sh disarm` for an explicit stop during your session. Its return code indicates whether the Pi confirmed zero output.

## Pair a new laptop explicitly on both sides

If the original session files are missing, or the laser does not reach this laptop despite working SSH, use the following pairing. It puts this laptop's actual IP in a new XML on the Pi and uses a distinct discovery tag. The previous Pi files and the original Mac pairing are preserved.

1. Ask the current operator to stop their mission and their manually launched hardware processes. If long-running drivers from a previous team session are still present, coordinate a normal reboot of the shared robot and let it reconnect. A reset of the ROS CLI daemon does not stop drivers. Do not run a separate bringup terminal.
2. From the ZIP root run

   ```bash
   bash ubuntu/robot.sh setup --dedicated
   bash ubuntu/robot.sh status
   bash ubuntu/robot.sh prepare
   ```

3. Repeat the scan check above and use the main README's navigation launcher. The dedicated files live in the `/home/pi/turtlebot_test_runs/ubuntu_exam_...` directory printed by setup. On another DHCP address, stop your session and coordinate a cold restart if its Pi services are still running. Run `setup --dedicated` again and restart your local ROS nodes so they use the new address.
4. To give the original Mac its turn, stop the Ubuntu mission, verify the Pi acknowledged zero, and coordinate a normal Pi reboot if the dedicated drivers are still running. The original Mac can then use its original launcher. To use the legacy session again **on Ubuntu**, run `bash ubuntu/robot.sh setup --legacy` after the old drivers have exited. A conflict, such as a guard PID from another session, is reported without terminating that process.

## Runtime files and dependencies

| Component | Expected on the Pi |
| --- | --- |
| ROS environment | `/opt/ros/humble/setup.bash`, `/home/pi/turtlebot3_ws/install/setup.bash`, `rmw_cyclonedds_cpp` |
| Base and lidar | `turtlebot3_bringup robot.launch.py`, `TURTLEBOT3_MODEL=waffle_pi`, `LDS_MODEL=LDS-01` |
| Camera | `turtlebot3_bringup camera.launch.py use_image_view:=false`, its camera driver, Python 3 with `cv2`, `numpy`, `rclpy`, ROS sensor messages |
| Guard | Archived `robot_velocity_guard.py`, ROS geometry, nav and sensor message packages, Python 3 |
| Original Pi session | `/home/pi/turtlebot_test_runs/robot103_2026-11-11_143310` with `robot-env.bash`, `robot_velocity_guard.py`, `pi_camera_jpeg.py` |
| Original Pi DDS XML | `/home/pi/turtlebot_test_runs/robot103_20261106/cyclonedds-robot.xml` |
| Dedicated setup | Creates an isolated Pi session from the two archived guard/JPEG scripts, a generated environment and XML. It uses the existing ROS installation and hardware workspace. |

If SSH times out, check whether .103 has completed boot and is on the same reachable LAN. If ROS says a PID uses a different `CYCLONEDDS_URI`, it found a running hardware program from a different setup. Coordinate with its owner and follow the handover procedure. Running a second hardware launch is not required. For local display without the Pi use the main README's offline preview, which is independent of robot pairing.
