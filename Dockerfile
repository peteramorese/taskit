# Dockerfile

# Base image
FROM ubuntu:20.04

# Set non-interactive mode during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary dependencies
RUN apt-get update && apt-get install -y \
    gnupg2 \
    lsb-release \
    wget \
    build-essential \
    vim

# Add the ROS GPG key and repository
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && \
    apt-key add ros.asc && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Update the package list again
RUN apt-get update

# Install ROS Noetic and rosdep
RUN apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Download and install CMake 3.20
RUN wget "https://cmake.org/files/v3.20/cmake-3.20.0-linux-x86_64.tar.gz" && \
    tar xf cmake-3.20.0-linux-x86_64.tar.gz && \
    cp -r cmake-3.20.0-linux-x86_64/bin/* /usr/local/bin/ && \
    cp -r cmake-3.20.0-linux-x86_64/share/* /usr/local/share/

# Install GCC 10
RUN apt-get install -y gcc-10 g++-10 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 \
                        --slave /usr/bin/g++ g++ /usr/bin/g++-10

# Clean up unnecessary files
RUN rm -rf ros.asc cmake-3.20.0-linux-x86_64.tar.gz cmake-3.20.0-linux-x86_64

# Set environment variables
ENV PATH="/usr/local/bin:${PATH}"
ENV ROS_DISTRO noetic

# Initialize ROS workspace
RUN apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool

# Create a catkin workspace
RUN mkdir -p /root/ws/src
WORKDIR /root/ws

RUN rosdep update && apt update && apt dist-upgrade

RUN apt install -y python3-wstool python3-catkin-tools python3-rosdep
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

#RUN wstool init src
#RUN wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
#RUN wstool update -t src
#RUN rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
#RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install compiler cache
RUN apt install ccache
RUN /bin/bash -c "echo 'export PATH=/usr/lib/ccache:$PATH' >> $HOME/.bashrc && source $HOME/.bashrc"

# Install all Dependency packages
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-moveit-visual-tools \
    ros-noetic-moveit-planners-ompl \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-moveit-ros-control-interface \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-vrpn-client-ros \
    ros-noetic-franka-control

COPY . /root/ws/src/taskit/

RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build the workspace
RUN catkin build 