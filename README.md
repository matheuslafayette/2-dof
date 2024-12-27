# 2-DOF Robot Arm Simulation

This project simulates a 2-DOF robotic arm using forward kinematics (FK), inverse kinematics (IK), and trajectory generation. It also includes visualization of trajectories and plots for analysis.

## Features
- **Forward Kinematics (FK):** Calculate the end-effector position given joint angles.
- **Inverse Kinematics (IK):** Calculate joint angles to reach a specific end-effector position.
- **Trajectory Generation:** Generate joint-space and Euclidean-space trajectories for smooth motion.
- **Visualization:** Create plots and animations of the robot's movements.

## Requirements
- Python 3.8 or higher
- Required Python libraries (listed in `requirements.txt`)

## Installation
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows, use venv\Scripts\activate
   ```

3. Install the required Python libraries:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
Run the main script to execute the tests and generate visualizations:
```bash
python main.py
```

### Outputs
- **Plots:** Trajectory plots are saved in the `plots` directory.
- **Animation:** A GIF of the robot's trajectory is saved in the `plots` directory as `RR.gif`.

## Team Members
 - [Matheus Lafayette](https://github.com/matheuslafayette/)
 - [Pedro Basílio](https://github.com/Pbgsa)
 - [Thiago Alves](https://github.com/Thijalves)
 - [Thiago Ramalho](https://github.com/Thiago-Ramalho)