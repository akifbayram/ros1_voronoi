# Voronoi-Based Multi-Robot Collaborative Exploration in Unknown Environments

This project customizes the code from the Voronoi paper repository: [Voronoi_Based_Multi_Robot_Collaborate_Exploration_Unknow_Enviroment](https://github.com/Peace1997/Voronoi_Based_Multi_Robot_Collaborate_Exploration_Unknow_Enviroment/tree/master).

## Host Machine Requirements
Before running the Docker container, ensure your host machine has the following:

### System Requirements
- Ubuntu 20.04 or later
- Docker Engine installed (version 20.10 or newer)

### Required Software
1. Docker Engine:
   ```bash
   sudo apt-get update
   sudo apt-get install docker-ce docker-ce-cli containerd.io
   ```

2. Rocker (for running GUI applications in Docker):
   ```bash
   sudo apt-get install python3-rocker
   ```

### Optional Requirements
- ROS2 (if using ros1_bridge functionality)
- Nvidia drivers (if using GPU acceleration)

## Docker Instructions

The Docker setup installs all required packages, dependencies, and Miniconda. With Docker, there is no need to run `catkin_make` or set up Conda separately.

### Building the Docker Image

In the root directory of the project repository, build the Docker image with:
```bash
docker build -t ros-custom-noetic .
```

### Running the Docker Container with Rocker

Launch the Docker container using Rocker with GUI support, network access, and shared memory:
```bash
sudo apt-get install python3-rocker
rocker --x11 --user --privileged \
    --volume /dev/shm:/dev/shm \
    --network=host \
    ros-custom-noetic tilix
```

This command opens a Tilix terminal session within the container.

## Experiment Instructions

### Launching Multi-Robot Exploration

1. **Launch 3 TurtleBots with Gmapping**:
   ```bash
   cd /voronoi_ws
   source ~/.bashrc
   source devel/setup.bash
   roslaunch multi_turtlebot3_explore three_turtlebot3_gmapping.launch
   ```

2. **Activate the Voronoi Environment**:
   ```bash
   source /opt/miniconda/etc/profile.d/conda.sh
   conda init
   conda activate voronoi
   ```

3. **Launch Voronoi Based Exploration**
   ```bash
   cd /voronoi_ws/src/voronoi/multi_turtlebot3_explore/scripts/turtlebot3_voronoi &&
   python3 dijkstra_test.py
   ```

3. **ros1_bridge**
   If used in conjunction with ROS2 on the host machine and `ros1_bridge`:
   ```bash
   source /opt/ros/noetic/setup.bash
   roscore
   ```