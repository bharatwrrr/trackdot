# LiDAR-Camera Fusion Package

This package overlays LiDAR point cloud data onto camera images.

## Configuration

Parameters are configured in `overlay_config.yaml`:

- **Camera Intrinsics**: `camera.fx`, `camera.fy`, `camera.cx`, `camera.cy`, `camera.width`, `camera.height`
- **Extrinsics**: `extrinsics.translation` (x, y, z), `extrinsics.rotation_rpy` (roll, pitch, yaw)
- **Topics**: Input camera/lidar topics and output overlay topic
- **Sync**: Maximum time difference for message synchronization

## Build

```bash
cd ~/trackdot
colcon build --packages-select lidar_cam_fusion
source install/setup.bash
```

## Run

Launch the node with configuration:

```bash
ros2 launch lidar_cam_fusion overlay.launch.py
```

Or run directly with parameters:

```bash
ros2 run lidar_cam_fusion lidar_camera_overlay_node --ros-args --params-file install/lidar_cam_fusion/share/lidar_cam_fusion/overlay_config.yaml
```

## Topics

**Subscribed:**
- Camera images (default: `/triton_left/images`)
- LiDAR points (default: `/lidar_points`)

**Published:**
- Overlay image (default: `/fusion/overlay_image`)
