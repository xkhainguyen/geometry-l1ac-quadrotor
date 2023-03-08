# L1 Adaptive Augmentation for Geometric Tracking Control of Quadrotors
## MATLAB simulation by [Khai Nguyen](https://www.linkedin.com/in/khainx/)

### Objective: Study and simulate this interesting work [L1 Adaptive Augmentation for Geometric Tracking Control of Quadrotors](https://arxiv.org/pdf/2109.06998.pdf)

![gif](./myfile.gif)

The main program `MainGeoL1_v1.m` includes three main sessions: (1) setup, (2) main loop, and (3) visualization
1. Setup:
    - This project is developed in MATLAB using _object-oriented programming (OOP)_. This is the first time I try OOP in MATLAB and it works well.
    - Classes: 
        - `Quadrotor`: defines quadrotor parameters and dynamics
        - `Geometry`: defines methods associated to cordinates
        - `Planner`: defines simulation scenarios (initial states, desired trajectories and disturbances). Scenario details are provided below.
        - `Controller`: defines geometric controller (baseline)
        - `Visualize`: defines methods to draw figures and animation for visualization
        - `L1AC`: defines L1 Adaptive Control algorithms
2. Main loop:
    - Step 1: Generate disturbances (time-continuous)
    - Step 2: Sample Ts
        - Calculate current desired trajectories
        - Calculate baseline control signal (Geometric Control)
        - Calculate adaptive control signal (L1AC)
        - Calculate total control input
    - Step 3: Apply control input to plant dynamics
    - Step 4: Return to Step 1 until finishing
3. Visualization
    - Draw figures on position tracking error, attitude tracking error, velocity tracking error, control input and disturbance estimate
    - Make animation of 3D quadrotor motion
    - Use `MakeVideos.m` to save gif 
### Scenarios
1. `Planner(0, x) (x is a unused arbitrary value)` presents a hovering quadrotor with one type of disturbance.
2. `Planner(1, x)` presents a quadrotor following a helix trajectory with disturbance dependent on `x`.
    - `x = 1`: No disturbance
    - `x = 2, 3, 4`: Different disturbances, see codes and the original paper for details
3. `Planner(2, x)` presents a quadrotor performing a flip with disturbance dependent on `x`.
    - `x = 1`: No disturbance
    - `x = 2, 3, 4`: Different disturbances, see codes and the original paper for details

Check out `Simulation Results.docx`

*Please give me a :star: if you find my repo useful!*
