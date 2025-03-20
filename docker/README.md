# Docker For gnssFGO

This is a dockerfile example based on ROS2 humble for using gnssFGO and adapted mapviz.

## How to build

1. clone the repository
2. cd gnssFGO/docker
3. run following
```bash
docker build -t haomingac/gnssfgo:latest .
```

## OR download the docker image
If you don't want to build you can download the example docker image here:
```bash
docker pull haomingac/gnssfgo:latest
```

## Run with docker compose

If you have `docker compose` installed, you can run it with `compose.yaml`. Most of settings in `compose.yaml` are for visualization.

1. clone the repository
2. cd gnssFGO/docker
3. build docker image. If you have built it, skip this step.
4. If you do not use Nvidia GPU, remove `deploy` part in `compose.yaml`.
5. run following to start the container
```bash
docker compose up -d
```
6. You can access the container interactively:
```bash
docker exec -it gnssfgo bash
```

If you got error on '' Authorization required, but no authorization protocol specified'', then start a terminal and run
```bash
xhost +
```

7. And compile the code
```
export CPATH="/opt/ros/humble/include/pcl_msgs:/opt/ros/humble/include/visualization_msgs"
cd /workspace/fgo_ws \
  && source /opt/ros/humble/setup.bash \
  && colcon build --cmake-args " -GNinja" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /workspace/mapviz_ws/install/setup.bash
source /workspace/fgo_ws/install/setup.bash
ldconfig
```
