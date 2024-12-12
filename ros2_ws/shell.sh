rocker --nvidia --x11 --privileged --name "ros2_rust_dev_container" --volume $(pwd):/workspace -- ros2_rust_dev /bin/bash || \
docker exec -it "ros2_rust_dev_container" bash
