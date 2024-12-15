# Using new messages

1. Update `Cargo.toml`
1. Update `package.xml`
1. In `ros2_ws` directory, run
   ```
   colcon build --packages-up-to macro_sim
   ```
1. In this directory, run
   ```
   cargo run
   ```

# Useful Commands
```
ros2 topic pub /rover_velocity_cmd geometry_msgs/Twist '{"linear":{"x": 1.0, "y":0.0, "z":0.0}, "angular":{"x": 0.0, "y": 0.0, "z": 1.0}}'
```
