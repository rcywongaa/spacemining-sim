# Asteroid Mining Simulation

## Overview
1. Excavator bot navigates (using a Nav2 plan) to mining site
1. Excavator bot starts mining
   1. MoveIt2 + Gazebo simulation plans and executes mining motion
   1. Upon success, Gazebo triggers Bevy to start spawning resources
1. Collector bot collects resources
1. Collector bot brings resources back to refinery

## Technologies Used
- Bevy (Rust)
- `ros2-rust`
- Space ROS
- Gazebo
- SMACC2
- BehaviorTree.CPP

## Macro Sim (Bevy)
### Entities
- Rovers
  - Base
  - Tool (inspired by [Offworld](https://www.offworld.ai/products))
    - Excavator
    - Collector
    - (Surveyor)
    - (Dozer)
    - (Hauler)
    - (Fixer)
- Mining site
- Refinery/Factory
- Solar Panels
- Command Center

### Components
- Health
- Tool Slot
- Capacity

### Systems
- Degradation
- Pickup Tool
- Drop Tool
- Tool Operation
- Natural Disaster
- Collisions
- Base motion
- Gravity

## Rovers (Space ROS)
### Topics
Publish:
- Battery Level
- Position

Subscribe:
- Costmap

### Services
- Set battery level

### Actions
- Go To Position
- Excavate / Operate Tool

### Hierarchical Finite State Machine Behavior Tree Hybrid (HFSMBTH)
https://www.youtube.com/watch?v=Qq_xX1JCreI&t=1167s

SMACC2 + BehaviorTree.CPP

States:
- Autonomous
  1. Go to worksite
  1. Do work
  1. Repeat
- Low Battery
  1. Go to charging station
  1. Charge
- Manual
  1. Execute
  1. Wait for X seconds
  1. Go back to Autonomous

## Micro Sim (Gazebo)
Use MoveIt2, Nav2 and Gazebo to simulate certain robot behaviors
- Drilling motion

## TODO
- [] Set up CI/CD & linters
- [x] Bare minimum macro sim
  - [x] Single excavator with manual controls
  - [x] Interface with ROS using `ros2-rust`
- [] Set up robot running Space ROS
  - [] Create HFSMBTH
- [] Set up micro sim (Gazebo)
- [] Set up MoveIt2
- [] Set up Nav2

## FAQ
Why Bevy and not Gazebo?
1. I wanted an excuse to learn the Bevy engine.
1. Radial (planetary) gravity
1. I plan to scale this to large number of robots and didn't want to simulate the physics of every single one.
   Note that Gazebo is still used when simulating the sensor and robot arm during arm motion planning.
1. [Gazebo vs *\<insert game engine\>*](https://discourse.ros.org/t/why-do-we-use-gazebo-instead-of-unreal-or-unity/25890/15).
