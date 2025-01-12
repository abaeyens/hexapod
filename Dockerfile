FROM osrf/ros:jazzy-desktop@sha256:2dad977b32fe39f7b5973f3cfb95ce5e7102bc71a25b382ec5d17b837839ebc2
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu


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
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Install other ROS-related packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-rqt* \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# Install other non-ROS packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    nodejs npm \
    python3-pip \
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
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME

# Give sudo privileges to the non-root user if needed
# RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME

# Set up .bashrc
RUN \
    # Add terminal coloring for the new user
    echo 'PS1="(container) ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/${USERNAME}/.bashrc && \
    # Disable "EasyInstallDeprecationWarning: easy_install command is deprecated"
    echo 'PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"' >> /home/$USERNAME/.bashrc  && \
    echo 'export PYTHONWARNINGS' >> /home/$USERNAME/.bashrc && \
    # Source ROS 2 setup
    echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

# Switch to the non-root user
USER $USERNAME

# Set entry point
CMD ["/bin/bash"]

