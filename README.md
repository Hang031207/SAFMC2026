# 🚁 SAFMC 2026 – High-Speed Drone Flock (D2)

Autonomous multi-drone system developed for the Singapore Amazing Flying Machine Competition (SAFMC) 2026.

This project implements a high-speed autonomous drone flock capable of navigating through designated gates while coordinating multiple drones safely and efficiently.


# Competition Overview

Competition: Singapore Amazing Flying Machine Competition (SAFMC) 2026  
Category: D2 – High-Speed Drone Flock

The objective of this category is to design and deploy 2–3 autonomous drones capable of:

- Performing fully autonomous flight
- Navigating through a sequence of designated gates
- Maintaining safe multi-drone operation
- Completing the course as fast and reliably as possible

The challenge combines high-speed control, trajectory planning, and multi-robot coordination.


# Project Goals

The main goals of this repository are:

- Implement autonomous gate navigation
- Enable multi-drone flock coordination
- Achieve high-speed trajectory tracking
- Ensure safe operation and collision avoidance

Key technical focuses include:

- Real-time trajectory generation  
- Multi-agent coordination  
- Obstacle and drone avoidance  
- Reliable flight control under high speed


# System Architecture



Main Modules

Trajectory Planning
- Generates smooth trajectories through gates
- Ensures speed constraints and feasibility

Artificial Potential Field (APF)
- Used for collision avoidance
- Handles drone–drone interaction and environmental constraints

Multi-Drone Coordination
- Leader–follower (breadcrumb trail trajectory)
- Communicates in MAVLink Messages through WiFi Router

Low-Level Flight Control
- Implemented through PX4 and MAVSDK


# Technologies Involved

- Python
- MAVSDK
- PX4 Autopilot
- NumPy / SciPy
- OpenCV (camera calibration & vision tasks)
- ROS / ROS2 (optional depending on setup)


# Features

- Autonomous gate navigation
- Multi-drone coordination
- Artificial Potential Field collision avoidance
- High-speed trajectory generation
- Simulation and trajectory visualization tools


# Testing and Simulation

Before real flight tests, trajectories and control strategies are verified using:

- Trajectory visualization tools
- Offline simulations
- Multi-drone coordination testing

These tools help ensure:

- Gate alignment correctness
- Collision avoidance effectiveness
- Feasible speed profiles


# Team

ICLAST in SAFMC 2026 – D2 High-Speed Drone Flock.

# References

- SAFMC Official Website: https://www.safmc.com.sg/about-the-competition/#cat-multi-machine
