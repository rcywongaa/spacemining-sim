rocker --nvidia --x11 --privileged --name "spaceros2_rust_dev_container" --user --user-override-name "spaceros-user" --user-preserve-home --user-override-shell '' --volume $(pwd):/home/spaceros-user/workspace -- spaceros2_rust_dev /bin/bash || \
docker exec -it "spaceros2_rust_dev_container" bash
