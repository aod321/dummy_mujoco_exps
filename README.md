# MuJoCo Robot Control Visualization

This project provides a visualization and control interface for robotic manipulation using MuJoCo physics engine. It features force/torque sensor visualization, Jacobian-based control, and compliance control capabilities.

## Prerequisites

### 1. MuJoCo Installation

Versioned releases are available as precompiled binaries from the GitHub [releases page](https://github.com/google-deepmind/mujoco/releases), built for Linux (x86-64 and AArch64), Windows (x86-64 only), and macOS (universal). This is the recommended way to use the software.

### Python (>= 3.9)

[](https://github.com/google-deepmind/mujoco#python--39)

The native Python bindings, which come pre-packaged with a copy of MuJoCo, can be installed from [PyPI](https://pypi.org/project/mujoco/) via:

```shell
pip install mujoco
```


## Usage

The main script `view_model.py` provides several command-line options for different functionalities:

```bash
python view_model.py [options]
```

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

Note: On macOS, replace `python` with `mjpython` in all commands below.

1. Basic visualization:

```bash
python view_model.py
```

2. Enable sensor data plotting:

```bash
python view_model.py --plot
```

3. Enable Jacobian control:

```bash
python view_model.py --jacobian --site left_gripper_site
```

4. Enable compliance control:

```bash
python view_model.py --compliance --force-gain 0.0001 --force-threshold 1
```

## Viewer Controls
- Left mouse button drag: Rotate view
- Right mouse button drag: Pan view
- Mouse wheel: Zoom view
- Ctrl+right mouse button drag: Apply external force to the model
