# Base image with ROS Noetic
FROM ghcr.io/sloretz/ros:noetic-simulators-osrf

# Copy workspace into the container
COPY . /voronoi_ws

ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages and dependencies
RUN apt-get update && \
    apt-get install -y \
        ros-noetic-multirobot-map-merge \
        ros-noetic-gmapping \
        ros-noetic-navigation \
        python3-empy \
        python3-numpy \
        tilix \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Set permissions on the workspace folder
RUN chown -R 1000:1000 /voronoi_ws

# Build the ROS workspace
WORKDIR /voronoi_ws

# Clear any existing build and devel directories to avoid cache conflicts
RUN rm -rf build devel
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Download and install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p /opt/miniconda && \
    rm miniconda.sh && \
    /opt/miniconda/bin/conda init && \
    echo "source /opt/miniconda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "conda activate base" >> ~/.bashrc

# Add Conda to PATH
# ENV PATH=/opt/miniconda/bin:$PATH

# Install necessary Python packages within the Conda environment
RUN /opt/miniconda/bin/pip install defusedxml rospkg

# Install the Conda environment from environment file
RUN /bin/bash -c "source /opt/miniconda/etc/profile.d/conda.sh && \
    conda env create -f /voronoi_ws/src/voronoi/environment_voronoi_python3_9_archlinux.yml"

# Switch to non-root user after build (optional)
USER 1000

# Set default command to open Tilix
CMD ["tilix"]
