# Instructions
1. Build `osrf/space-ros:latest` container with `space-ros/build.sh`
1. Build `spaceros2_rust_dev` container with `space_build.sh`
1. Enter container with `space_shell.sh`
1. Build `ros2_rust` and `macro_sim` with `colcon build --packages-up-to macro_sim`
   May need
   ```
   colcon build --allow-overriding action_msgs builtin_interfaces geometry_msgs rcl_interfaces rosgraph_msgs rosidl_default_generators rosidl_default_runtime std_msgs test_interface_files test_msgs unique_identifier_msgs --packages-up-to macro_sim
   ```

# Caveats
Make sure msg files between `ros2-rust` and `spaceros` are compatible
```
--allow-overriding action_msgs builtin_interfaces geometry_msgs rcl_interfaces rosgraph_msgs rosidl_default_generators rosidl_default_runtime std_msgs test_interface_files test_msgs unique_identifier_msgs
```

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
