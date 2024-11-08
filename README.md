This C++ program simulates the flight and impact of a rocket, incorporating PID (Proportional, Integral, Derivative) control for yaw adjustments. The simulation accounts for gravitational effects and allows for launches from various initial Y positions, including zero or below.

Key Features
PID Control:

The PIDController class implements a PID controller, which calculates corrections for the yaw angle of the rocket based on the error between the target yaw and the current yaw. This ensures that the rocket adjusts its trajectory to hit the desired target.

Trajectory and Impact Calculation:

The RocketSimulation class handles the physics of the rocket's flight. It includes a method calculateImpactPoint that calculates the estimated impact point based on initial conditions and the target yaw angle. This method correctly handles cases where the initial Y position is zero or lower.

Simulation Loop:

The runSimulation method runs a simulation of the rocket's flight. The method updates the rocket's position and velocity over time, applying gravitational effects and yaw corrections. The simulation continues until the rocket reaches its peak height and starts descending, after which it is considered to have hit the ground.

User Input:

The main function collects input from the user, including the initial position, speed, launch angle, initial yaw, and target yaw. These inputs are used to initialize the simulation and calculate the rocket's trajectory and impact.

Example Usage
The user inputs the initial X, Y, and Z positions, the initial speed in meters per second, the launch angle in degrees from the horizontal, the initial yaw angle in degrees, and the target yaw angle in degrees.

The simulation calculates the estimated impact point and prints it.

After a brief delay, the simulation starts and prints the state of the rocket at each time step, including position, velocity, yaw, and speed.

The simulation ends when the rocket hits the ground.

Code Structure
PIDController: Handles PID calculations for yaw adjustments.

RocketSimulation: Manages the rocket's flight physics, trajectory calculations, and simulation loop.

main: Collects user input and initiates the simulation.

This code provides a comprehensive simulation of rocket flight, incorporating both physics and control theory to accurately predict and adjust the rocket's trajectory.
