# Two-Link Robotic Arm Simulation and Control

A Python-based simulation of a two-link planar robotic manipulator with two revolute joints. This project explores robotic kinematics, workspace analysis, trajectory planning, nonlinear dynamics, and feedback control.

## Project Objectives

- Develop forward and inverse kinematics.
- Analyse the reachable workspace.
- Visualize and animate robotic arm configurations.
- Implement joint-space and Cartesian trajectory planning.
- Develop a nonlinear dynamic model.
- Implement PID-based feedback control.
- Evaluate trajectory tracking performance.

## Technology Stack

- Python
- NumPy
- Matplotlib
- SciPy (planned)
- pytest
- Git and GitHub

## Current Implementation

### Phase 1: Forward Kinematics
- Analytical forward-kinematics equations.
- Calculation of joint and end-effector positions.
- Modular Python implementation.
- Analytical validation tests.

### Phase 2: Inverse Kinematics
- Analytical inverse-kinematics solver.
- Elbow-up and elbow-down configurations.
- Unreachable target detection.
- Forward-kinematics consistency verification.

### Phase 3: Workspace and Visualization
- Reachable workspace generation.
- Robotic arm visualization.
- Joint-angle animation.

*Visualization features are currently under development and validation.*

## Installation

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/two-link-robotic-arm.git
cd two-link-robotic-arm
```

Create a virtual environment:

```bash
python -m venv .venv
```

Activate it on Windows:

```powershell
.venv\Scripts\activate
```

Install the dependencies:

```bash
pip install -e ".[dev]"
pip install matplotlib
```

## Running Examples

Forward kinematics:

```bash
python examples/forward_kinematics_demo.py
```

Inverse kinematics:

```bash
python examples/inverse_kinematics_demo.py
```

Workspace visualization:

```bash
python examples/workspace_demo.py
```

Robotic arm animation:

```bash
python examples/animation_demo.py
```

## Running Tests

```bash
python -m pytest -v
```

## Project Parameters

| Parameter | Value |
|---|---:|
| Link 1 length | 0.5 m |
| Link 2 length | 0.4 m |
| Link 1 mass | 1.0 kg |
| Link 2 mass | 0.8 kg |
| Gravity | 9.81 m/s² |

## Future Development

- Cubic trajectory generation.
- Cartesian trajectory planning.
- Euler–Lagrange dynamic modelling.
- Numerical integration.
- PID feedback control.
- Performance evaluation and automated validation.

## Author

Mechanical Engineering Student  
Dr. B.R. Ambedkar National Institute of Technology, Jalandhar