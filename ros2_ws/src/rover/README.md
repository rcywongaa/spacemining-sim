# Rover

## FSM Dreamlist
<https://github.com/erikzenker/hsm>
<https://smacc.dev/statechart-vs-msm/>

Must haves:
- Superstate enter/exit actions
- Direct transitions to/from substates (which should trigger the appropriate superstate enter/exit actions)
- Events sent to both super state and leaf state (super state overrides leaf state transitions)
Good to haves:
- Per state member variables
- Per state transition tables

## FSM Decision
- ~~TinyFSM~~: No support for hierarchical FSM
- ~~SML~~: Hacky solutions for direct transitions to/from substates
- ~~SMACC2~~: [Only leaf states can contain orthogonals](https://smacc.dev/hierarchical-states/) means superstates cannot have enter/exit actions.
- ~~HFSM2~~: Superstates don't receive events
- ~~MSM~~: Doesn't scale to large state machines, also same problem as SML
- Statecharts

## States
- Autonomous
  - On nav target:
    - Transition to Manual
  - Working
  - Charging
- Manual
  - On Enter: Start Timer
  - On timer expire:
    - Transition to autonomous
  - Substates:
    - Go to target
- Sleep
## Useful Commands
```
ros2 service call /send_nav_target rover_interfaces/srv/SendNavTarget '{"target": {"position": {"latitude": 50, "longitude": 50, "altitude": 0}, "heading": 0}}'
```
