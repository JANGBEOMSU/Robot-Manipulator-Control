# Robot Manipulator Control

Python examples for the kinematics and visualization of a three-degree-of-freedom robot manipulator.

## Features

- Denavit–Hartenberg homogeneous transformation
- Forward kinematics
- Analytical inverse kinematics
- Jacobian-transpose inverse kinematics
- Coordinate-frame transformation
- 3D robot-arm visualization

## Project structure

```text
Robot-Manipulator-Control/
├── README.md
├── requirements.txt
├── src/
│   └── robot_manipulator/
│       ├── __init__.py
│       ├── coordinate_transform.py
│       ├── inverse_kinematics.py
│       ├── kinematics.py
│       └── velocity_kinematics.py
└── examples/
    ├── analytical_ik_cli.py
    ├── coordinate_transform_cli.py
    ├── forward_kinematics_cli.py
    ├── robot_arm_3d_visualization.py
    ├── target_pose_visualization.py
    └── velocity_ik_cli.py
```

## Installation

```bash
python -m venv .venv
```

Windows:

```bash
.venv\Scripts\activate
pip install -r requirements.txt
set PYTHONPATH=src
```

macOS/Linux:

```bash
source .venv/bin/activate
pip install -r requirements.txt
export PYTHONPATH=src
```

## Example usage

```bash
python examples/forward_kinematics_cli.py
python examples/analytical_ik_cli.py
python examples/velocity_ik_cli.py
python examples/robot_arm_3d_visualization.py
```

## Robot parameters

- Link 2 length: `0.5 m`
- Link 3 length: `0.3 m`
- Joint 1 limit: `-180° to 180°`
- Joint 2 limit: `-90° to 90°`
- Joint 3 limit: `-90° to 90°`
