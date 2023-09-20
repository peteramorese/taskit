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
    vim \
    python3-pip \
    apt-utils flex bison mona 

# Download and install CMake 3.20
RUN wget "https://cmake.org/files/v3.20/cmake-3.20.0-linux-x86_64.tar.gz" && \
    tar xf cmake-3.20.0-linux-x86_64.tar.gz && \
    cp -r cmake-3.20.0-linux-x86_64/bin/* /usr/local/bin/ && \
    cp -r cmake-3.20.0-linux-x86_64/share/* /usr/local/share/

# Install GCC 10
RUN apt-get install -y gcc-10 g++-10 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 \
                        --slave /usr/bin/g++ g++ /usr/bin/g++-10


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

RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
RUN tar -xzvf eigen-3.4.0.tar.gz
RUN ls
RUN cd eigen-3.4.0 && mkdir build && cd build
WORKDIR /eigen-3.4.0/build
RUN cmake -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc ..
RUN make && make install

# Clean up unnecessary files
RUN rm -rf ros.asc cmake-3.20.0-linux-x86_64.tar.gz cmake-3.20.0-linux-x86_64

# Install spot using apt
RUN wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | apt-key add -
SHELL ["/bin/bash", "-c"] 
RUN pip3 install pyyaml numpy bidict networkx graphviz ply pybullet pyperplan==1.3 cython IPython svgwrite matplotlib imageio lark-parser==0.9.0 sympy==1.6.1
RUN echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list
RUN apt-get -y update && apt -y install spot \
    libspot-dev \
    spot-doc

# Set environment variables
ENV PATH="/usr/local/bin:${PATH}"
ENV ROS_DISTRO noetic

# Initialize ROS workspace
RUN apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool

# Create a catkin workspace
RUN mkdir -p /root/ws/src
WORKDIR /root/ws

RUN rosdep update && apt update && apt dist-upgrade -y

RUN apt install -y python3-wstool python3-catkin-tools python3-rosdep
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

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
    ros-noetic-franka-control \
    ros-noetic-panda-moveit-config

# Install required libraries for Qt X11
RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libqt5gui5 \
    libqt5widgets5 \
    x11-apps

## Install grapefruit_ros
#RUN git clone https://github.com/peteramorese/grapefruit_ros.git /root/ws/src/grapefruit_ros
#
#WORKDIR /root/ws/src/grapefruit_ros
#RUN git submodule update --init --recursive
WORKDIR /root/ws

ADD ./ /root/ws/src/taskit/

RUN catkin config --install --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release 

# Build the workspace
RUN catkin build
RUN echo 'source ~/ws/devel/setup.bash' >> ~/.bashrc

### Adding aliases 

# Run robot with vicon
RUN echo 'alias run_robot_w_vicon="roslaunch taskit manipulator_node.launch sim:=false pose_tracker:=vrpn"' >> ~/.bashrc

# Run robot without vicon
RUN echo 'alias run_robot_wo_vicon="roslaunch taskit manipulator_node.launch sim:=false"' >> ~/.bashrc

# Run sim with vicon
RUN echo 'alias run_sim_w_vicon="roslaunch taskit manipulator_node.launch sim:=true pose_tracker:=vrpn"' >> ~/.bashrc

# Run sim without vicon
RUN echo 'alias run_sim_wo_vicon="roslaunch taskit manipulator_node.launch sim:=true"' >> ~/.bashrc