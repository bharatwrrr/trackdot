## Starting a Data Collection Run

Run these steps in order each time:

```bash
# 1. Set up ROS2 domain IDs and DDS
source ~/ros2_configs/config.env

# 2. Configure network interfaces, camera IPs, and PTP and then wait for 10-20 seconds for PTP
sudo ~/trackdot/src/bringup/scripts/setup_network.sh

# 3. Build and source (only needed after code changes)
cd ~/trackdot
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# NOTE: if colcon build fails, use --packages-select flag and list down the required pacakges (camera driver, lidar driver, CAN, GNSS, bringup)
source install/setup.bash

# 4. Launch all sensors and start recording
ros2 launch bringup data_collection.launch.py
```

Bags are saved to `/home/bharatwrrr/test_run/bags/` with the prefix `trackdot_<timestamp>`.
The bag name prefix and output directory are in `src/bringup/config/data_collection_params.yaml`.

Wait for both cameras to print `PTP Slave lock acquired` (~10–20 s) before driving — recording starts automatically as soon as the launch is up, but camera frames are not published until PTP sync completes.

Press `Ctrl-C` to stop recording and shut down all nodes.

## Network Setup

## Cameras

The Triton cameras connect via PoE through a Teltonika TSW200 switch. `setup_network.sh` starts a PTP grandmaster (`ptp4l`) on `eno1` so both cameras sync their clocks to Jetson time on every boot.

When launching `triton_cam_driver`, the driver waits for each camera to reach PTP Slave state before streaming (~10–20 s). You will see:

```
[triton_left]: PTP status: Uncalibrated — waiting for Slave...
[triton_left]: PTP Slave lock acquired — camera clock stepped to Unix time.
```

Both cameras must complete this before data collection begins.

## Atlas LiDAR

The Atlas connects directly to the Jetson's USB 3.2 port via gigabit Ethernet (avoiding PoE congestion). Use `aevacli` to configure it:

```bash
# View current config
aevacli configuration show --sensor-url 10.42.0.45

# Set scan pattern (e.g., to Automotive-Wide1)
aevacli configuration set --sensor-url 10.42.0.45 --group scanner --setting scan_pattern=Automotive-Wide1 -t 5
```

The `-t 5` timeout flag is necessary because configuration changes trigger a LiDAR reboot. You may see a TIMEOUT error the first time you run a command—this is expected and the setting typically applies anyway. Verify with `configuration show`.

The `setup_network.sh` script handles configuration setup automatically. Ensure the LiDAR's host IP is set to the Jetson's IP.

## Inspecting a Recorded Bag

```bash
source ~/trackdot/install/setup.bash
ros2 bag info <bag_dir>          # topics, message counts, duration
ros2 bag play <bag_dir>          # replay
```

If `ros2 bag info`/`play` fails with `Could not find metadata in bag directory`, the recorder didn't shut down cleanly (crash, kill, power loss) and never wrote `metadata.yaml`. Fix it once with:

```bash
ros2 bag reindex -s mcap <bag_dir>
```

This does a full linear scan of the `.mcap` file to rebuild the index, so for a large bag it can take a couple of minutes — it only needs to be run once per bag. **Run it in a detached/background shell carefully**: wrapping it in `setsid` breaks the usual job-tracking of some terminal/agent harnesses (it silently orphans the process from the shell that launched it, which can make the launcher misreport the job as finished early). Plain `&`/`nohup` without `setsid` is safer, or just run it in the foreground and wait.

### Extracting vehicle speed (or other topics) from a bag

`scripts/extract_vehicle_speed.py` pulls `/vehicle_state.speed_mps` out of a bag and prints min/max/mean, with an optional CSV dump:

```bash
source ~/trackdot/install/setup.bash
python3 src/bringup/scripts/extract_vehicle_speed.py <bag_dir> --csv speed.csv
```

It works on a normal (reindexed) bag via `rosbag2_py`, **and** on a bag that still has no `metadata.yaml`, by falling back to a raw linear scan of the `.mcap` file with the `mcap` Python library — no reindexing required for this script specifically. That fallback path needs two extra pip packages that aren't part of the ROS2 install:

```bash
pip3 install --user -r src/bringup/scripts/requirements.txt
```

The same raw-scan pattern in `read_via_raw_mcap()` works for any other topic (camera `/triton_*/images/compressed`, LiDAR `/aeva/atlas/point_cloud_compensated`, GNSS `/gnss/*`) — swap `TOPIC` and the message type at the top of the script.
