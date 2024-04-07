FROM ubuntu:22.04@sha256:aa772c98400ef833586d1d517d3e8de670f7e712bf581ce6053165081773259d


# ROS2
## Install prerequisites
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    curl wget gnupg2 lsb-release \
    software-properties-common
## Enable required repositories
RUN add-apt-repository universe
## Add ROS 2 GPG key
RUN apt update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
## Add the repo to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
## Install ROS 2 including dev tools
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-iron-desktop

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    gz-harmonic

# Install other ROS-related packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-iron-ros-gzharmonic \
    ros-iron-rqt* \
    ros-iron-plotjuggler-ros \
    ros-iron-xacro \
    ros-iron-joint-state-publisher \
    ros-iron-joint-state-publisher-gui

# Install other non-ROS packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-pip \
    gdb

# Install Python packages
RUN pip install \
    rockit-meco
RUN pip install \
    notebook \
    jupytext

# Clear apt cache
RUN rm -rf /var/lib/apt/lists/*


# Create a non-root user
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME

# Give sudo privileges to the non-root user if needed
# RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME

# Set up .bashrc
## Add terminal coloring for the new user
RUN echo 'PS1="(container) ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/${USERNAME}/.bashrc
## Source ROS 2 setup
RUN echo "source /opt/ros/iron/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

# Switch to the non-root user
USER $USERNAME

# Set entry point
CMD ["/bin/bash"]

