ARG ROS_DISTRO=jazzy

FROM osrf/ros:$ROS_DISTRO-desktop
RUN if [ "$ROS_DISTRO" = "jazzy" ]; \
    then touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu; \
    fi


# Install prerequisites
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    curl wget gnupg2 lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install dev tools and Gazebo Harmonic
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-$ROS_DISTRO-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Install other ROS-related packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install other non-ROS packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    nodejs npm \
    python3-pip \
    sudo \
    vim \
    gdb \
    && rm -rf /var/lib/apt/lists/*
RUN npm i -g \
    xunit-viewer

# Install Python packages
# TODO use something better than "break-system-packages"
RUN pip install --no-cache-dir --break-system-packages \
    rockit-meco \
    notebook \
    ipympl \
    jupytext


# Create a non-root user
ARG USER
ARG USER_ID
ARG GROUP_ID
RUN groupadd --gid $GROUP_ID $USER && \
    useradd --uid $USER_ID --gid $GROUP_ID --create-home --shell /bin/bash $USER

# Give sudo privileges to the non-root user if needed
RUN echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USER

# Set up .bashrc
RUN \
    # Add terminal coloring for the new user
    echo 'PS1="(container) ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/$USER/.bashrc && \
    # Disable "EasyInstallDeprecationWarning: easy_install command is deprecated"
    echo 'PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"' >> /home/$USER/.bashrc  && \
    echo 'export PYTHONWARNINGS' >> /home/$USER/.bashrc && \
    # Source ROS 2 setup
    echo "source /opt/ros/jazzy/setup.bash" >> /home/$USER/.bashrc && \
    echo "source install/setup.bash" >> /home/$USER/.bashrc

# Switch to the non-root user
USER $USER

# Set entry point
CMD ["/bin/bash"]

