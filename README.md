# nemo_gstream_filter_wrapper

ROS2 Python package for underwater ROV camera streaming with real-time image enhancement and ArUco marker detection.

## Overview

This package provides a complete video streaming pipeline for underwater ROV operations, designed for the **TAC 2026** competition. It captures video from cameras on the ROV (Jetson side), streams over UDP to a surface station (laptop), and applies real-time underwater image enhancement along with ArUco marker and pipeline detection.

## Usage

```bash
### bottom camera
# Default: port 5000
ros2 run nemo_gstream_filter_wrapper transmitter

ros2 run nemo_gstream_filter_wrapper receiver

### front camera
ros2 run nemo_gstream_filter_wrapper receiver1 --ros-args -p device:=/dev/video3

ros2 run nemo_gstream_filter_wrapper receiver --ros-args -p port:=5001
```

## Features

- **Real-time H.264 streaming** over UDP using GStreamer
- **8 underwater image enhancement fixes**:
  1. Curved acrylic blur correction (deblur kernel + unsharp mask)
  2. Blue/green color cast removal (gray-world white balance + red boost)
  3. Low contrast/haze correction (CLAHE on LAB luminance)
  4. Backscatter particle removal (median filter)
  5. Low brightness correction (gamma correction)
  6. Barrel distortion correction (undistort remap)
  7. Motion blur compensation (adaptive sharpening)
  8. Turbid water mode (aggressive CLAHE + dark-channel dehaze)
- **ArUco marker detection** for pipeline navigation and docking
- **Pipeline color segmentation** for visual tracking
- **Heads-up Display (HUD)** with telemetry overlay
- **TAC mode toggle** for turbid harbour water conditions

## Package Structure

```
nemo_gstream_filter_wrapper/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── nemo_gstream_filter_wrapper
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── nemo_gstream_filter_wrapper/
    ├── __init__.py
    ├── transmitter.py      # Jetson-side video streaming node
    ├── receiver.py          # Surface station receiver with enhancements
    ├── transmitter1.py     # Secondary camera transmitter
    ├── receiver1.py        # Secondary camera receiver
    └── underwater_enhance.py  # Shared image enhancement module
```

## Dependencies

- **ROS2** (tested with Humble/Iron)
- **Python 3.8+**
- **GStreamer 1.0** (`python3-gi`, `gir1.2-gstreamer-1.0`)
- **OpenCV** (built with GStreamer support)
- **NumPy**

Install GStreamer bindings:
```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
```

## Building

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/YOUR_USERNAME/nemo_gstream_filter_wrapper.git

# Build
cd ..
colcon build --packages-select nemo_gstream_filter_wrapper

# Source
source install/setup.bash
```

## Usage

p bitrate_kbps:=5000
```

**Transmitter Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `/dev/video0` | V4L2 device path |
| `host` | `192.168.2.1` | Destination IP address |
| `port` | `5000` | Destination UDP port |
| `width` | `640` | Video width |
| `height` | `480` | Video height |
| `fps` | `30` | Frame rate |
| `bitrate_kbps` | `5000` | H.264 encoding bitrate |
| `speed_preset` | `veryfast` | x264 encoding speed preset |
| `gst_brightness` | `0.05` | GStreamer-side brightness lift |
| `gst_contrast` | `1.10` | GStreamer-side contrast boost |

### Receiver (Laptop/Surface side)

Receives the stream, applies enhancements, and runs detection:



**Receiver Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `5000` | UDP port to listen on |
| `latency_ms` | `10` | RTP jitter buffer latency |
| `sync` | `false` | Sync mode |

**Runtime Controls:**
- Press `Q` to quit
- Press `T` to toggle TAC mode (turbid harbour dehazing)

## Camera Calibration

The package includes default calibration parameters for:
- **Camera 0**: Groov-e (`/dev/video0`, port 5000)
- **Camera 1**: Logitech C270 (port 5001)

Update the calibration matrices in `underwater_enhance.py` after running your own checkerboard calibration:

```python
# In underwater_enhance.py
CAM0_MATRIX = np.array([
    [fx,  0.0, cx],
    [0.0, fy,  cy],
    [0.0, 0.0, 1.0],
], dtype=np.float32)

CAM0_DIST = np.array([k1, k2, p1, p2, k3], dtype=np.float32)
```

## Enhancement Pipeline

The `underwater_enhance.py` module applies fixes in this order:

1. **Undistort** - Barrel distortion correction
2. **Backscatter removal** - Median filter for particles
3. **White balance** - Gray-world + red channel boost
4. **Gamma correction** - Brightness lift (γ=1.5)
5. **CLAHE** - Contrast limited adaptive histogram equalization
6. **Dehaze** (TAC mode only) - Dark-channel prior dehazing
7. **Deblur** - Laplacian-based acrylic blur correction
8. **Adaptive sharpen** - Motion blur compensation

## ArUco Detection

The receiver detects ArUco markers from dictionaries 4x4, 5x5, 6x6, and 7x7. Special marker IDs trigger pipeline navigation events:

| Marker ID | Role | Description |
|-----------|------|-------------|
| 56 | PIPELINE ENTRY | Start of pipeline inspection |
| 5 | SECTION 2 | Proceed forward |
| 20 | SECTION 3 | Scan surroundings |
| 32 | PIPELINE EXIT | Inspection complete |

