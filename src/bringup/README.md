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
