# Asteroid Mining Simulation

This is a proof of concept project that combines multiple technologies in the context of a simulated multi-robot asteroid mining scenario.

Robot Types: (inspired by [Offworld](https://www.offworld.ai/products))
  - Excavator
  - Collector
  - (Surveyor)
  - (Dozer)
  - (Hauler)
  - (Fixer)

## Mission Plan
1. Excavator bot navigates to mining site using Nav2
1. Excavator bot starts mining
   1. MoveIt2 + Gazebo simulation plans and executes mining motion
   1. Upon success, Gazebo triggers Bevy to start spawning resources
1. Collector bot collects resources
1. Collector bot brings resources back to refinery
1. Rovers automatically goes to charging station when low battery.

## Technologies Used
- [x] [Bevy](https://bevyengine.org/)
- [x] [`ros2-rust`](https://github.com/ros2-rust/ros2_rust)
- [x] [Space ROS](https://space.ros.org/)
- [x] [Hierarchical Finite State Machine Behavior Tree Hybrid (HFSMBTH)](https://www.youtube.com/watch?v=Qq_xX1JCreI&t=1167s)
  - [x] [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
  - [x] [HFSM2](https://github.com/andrew-gresyk/HFSM2)
- [ ] Gazebo
- [ ] Nav2
- [ ] MoveIt2

## TODO
- [ ] Set up CI/CD & linters
- [x] Bare minimum macro sim
  - [x] Single excavator with manual controls
  - [x] Interface with ROS using `ros2-rust`
- [x] Set up robot running Space ROS
- [x] Create HFSMBTH
  - [x] Create FSM
  - [x] Create BT
- [ ] Macro Sim UI
- [ ] Multiple rovers
- [ ] Set up micro sim (Gazebo)
- [ ] Set up MoveIt2
- [ ] Set up Nav2

## Macro Sim (Bevy + `ros2-rust`)
The asteroid and rover kinematics is simulated using Bevy, a popular Rust game engine that uses an ECS architecture similar to Gazebo.

The asteroid is treated as a sphere and the rover is assumed to be constrained to the surface of the asteroid.
The rover velocity is forward integrated and reprojected onto the surface of the sphere via geodesic calculations.

## Rovers (Space ROS + HFSMBTH)
The rover is controlled using Space ROS to mimic what a real space rover would use.

The behavior is coordinated using a HFSMBTH which aims to combine the benefits of hierarchical finite state machines (HFSM) and behavior trees (BT).

## FAQ
Why Bevy and not Gazebo?
1. I wanted an excuse to learn the Bevy engine.
1. Radial (planetary) gravity
1. I plan to scale this to large number of robots and didn't want to simulate the physics of every single one.
   Note that Gazebo will still be used when simulating the sensor and robot arm during arm motion planning.

Why HFSMBTH?
- (Reiterating key points from talk linked above)

  BTs are great at modelling sequences of fallible actions but awful at modelling cyclical behavior (BTs are directed acyclic graphs).

  FSMs are great at modelling cyclical behavior but awful at modelling sequences of fallible actions (fallback/retry adds lots of unnecessary states).

  Combine them both to get the best of both worlds.

Why HFSM2 and not TinyFSM/SML/SMACC2/Statechart/MSM?
- I wanted the following from my hierarchical FSM framework:
  - Superstate enter/exit actions
  - Direct transitions between substates
  - Per state member variables
  - Superstate initiated transitions
  - Transitions with payload
  Only HFSM2 satisfies all requirements.

  |                                      | ~~TinyFSM~~ (Not HSM)     | SML   | SMACC2 | HFSM2                      | ~~MSM~~ (superceded by SML)     | Statecharts                                            |
  | ------------------------------------ | --------------------- | ----- | ------ | -------------------------- | --------------------------- | ------------------------------------------------------ |
  | Superstate enter/exit actions        |                       | Y     | N      | Y                          | Y                           |                                                        |
  | Direct transitions to/from substates |                       | Hacky |        | Y                          |                             | Y                                                      |
  | Per state member variables           |                       | Y     | Y      | Y                          | ?                           |                                                        |
  | Superstates initiated transitions    |                       |       |        | Y                          |                             |                                                        |
  | Transition with payload              |                       |       |        | Y                          |                             | N (Parameterized states can act as hardcoded payloads) |

