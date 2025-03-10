# Instructions
1. Build `osrf/space-ros:latest` container with `space-ros/build.sh`
1. Build `spaceros2_rust_dev` container with `space_build.sh`
1. Enter container with `space_shell.sh`
1. Build with `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --packages-up-to bringup`
   May need
   ```
   colcon build --allow-overriding action_msgs builtin_interfaces geometry_msgs rcl_interfaces rosgraph_msgs rosidl_default_generators rosidl_default_runtime std_msgs test_interface_files test_msgs unique_identifier_msgs
   ```
# Launch
```
ros2 launch bringup bringup_launch.py
```

# Run Tests
```
colcon test-result --delete-yes && colcon test --ctest-args tests --packages-select rover_fsm && colcon test-result --all --verbose
```
# Conventions
`heading` is counterclockwise, `azimuth` is clockwise, both are between [-pi, pi].

# Notes
Make sure msg files between `ros2-rust` and `spaceros` are compatible

SpaceROS in general does not use ROS packages available from the package manager (`apt`).
Instead, the dependencies are cloned and built as part of the image.

# Troubleshooting
- ```
  ERROR:colcon.colcon_cmake.task.cmake.build:Failed to find the following files:
  - /home/spaceros-user/workspace/install/rosidl_runtime_rs/share/rosidl_runtime_rs/package.sh
  Check that the following packages have been built:
  - rosidl_runtime_rs
  ```
  Make sure `cargo` works (cargo/rust installed correctly and also entering container as the correct user)

- ```
  ld: cannot find -lrmw_implementation: No such file or directory
  ```
  This is due to SpaceROS not providing the default `rmw_implementation`.
  Fix this by changing `rclrs/build.sh` to use the specific RMW implementation provided by SpaceROS (`rmw_cyclonedds_cpp`)
  Replace
  ```
  println!("cargo:rustc-link-lib=dylib=rmw_implementation");
  ```
  with
  ```
  println!("cargo:rustc-link-lib=dylib=rmw_cyclonedds_cpp");
  ```
- ``error: linking with `cc` failed: exit status: 1``

  Perform a clean rebuild with `rm -rf build install log .cargo`.
  Also make sure `$LD_LIBRARY_PATH` ONLY contains stuff from the `/opt/ros/` directory and nothing from the current `install` directory.
