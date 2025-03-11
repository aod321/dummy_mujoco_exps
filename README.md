# MuJoCo Robot Control Visualization

This project provides a visualization and control interface for robotic manipulation using MuJoCo physics engine. It features force/torque sensor visualization, Jacobian-based control, and compliance control capabilities.

## Prerequisites

### 1. MuJoCo Installation

```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-macos-universal2.dmg # macOS
```
```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-linux-x86_64.tar.gz # Linux
```
```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-windows-x86_64.zip # Windows
```

### 2. Python (>= 3.9)

[](https://github.com/google-deepmind/mujoco#python--39)

After installing the MuJoCo binary package, the Python bindings can be installed from [PyPI](https://pypi.org/project/mujoco/) via:



## Usage

The main script `view_model.py` provides several command-line options for different functionalities:

### Command Line Options

- `--model`: Path to the MuJoCo model file (default: 'mjmodel/bimanual_dummy_transfer_cube.xml')
- `--duration`: Simulation duration in seconds, 0 for unlimited (default: 0)
- `--plot`: Enable sensor data text visualization
- `--jacobian`: Enable Jacobian-based end-effector pose control
- `--site`: End-effector site name for Jacobian control (default: 'left_gripper_site')
- `--delta`: End-effector pose increment size (default: 0.001)
- `--compliance`: Enable compliance control based on force sensors
- `--force-gain`: Force sensor gain coefficient (default: 0.01)
- `--force-threshold`: Force sensor threshold (default: 0.5)

### Example Commands

Note: On **macOS**, replace `python` with `mjpython` in all commands below.

1. Basic visualization:

```bash
python view_model.py
```
or in macOS:
```bash
mjpython view_model.py
```

2. Enable sensor data plotting:

```bash
python view_model.py --plot
```

3. Enable compliance control:

```bash
python view_model.py --compliance --force-gain 0.0001 --force-threshold 1
```

## Viewer Controls
- Left mouse button drag: Rotate view
- Right mouse button drag: Pan view
- Mouse wheel: Zoom view
- Ctrl+right mouse button drag: Apply external force to the model


## Known Issues and Future Improvements

### Gravity Compensation
The current implementation simply cancels all gravity components, which also ignores vertical dragging forces. A more sophisticated approach would calculate precise gravity components based on real-time pose:

- Implement real-time gravity decomposition based on end-effector orientation
- Only compensate for the actual gravity component while preserving user-applied forces
- Add configuration option to adjust gravity compensation sensitivity


