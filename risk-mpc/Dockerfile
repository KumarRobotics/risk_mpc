# The upstream image depends on the architecture, and it is defined when
# building the image. Please see build.bash for more details
ARG UPSTREAM_IMAGE
FROM $UPSTREAM_IMAGE
ARG ARCHITECTURE
ARG BARE_OR_NVDA

#Run the frontend first so it doesn't throw an error later
RUN apt-get update \
  && export TZ="America/New_York" \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y locales \
  && ln -fs "/usr/share/zoneinfo/$TZ" /etc/localtime \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && apt-get clean

# General dependencies for development
RUN apt-get update \
  && apt-get install -y --install-recommends\
  build-essential \
  cmake \
  coinor-libipopt-dev \
  cppcheck \
  curl \
  dbus-x11 \
  g++ \
  gcc \
  gdb \
  gfortran \
  git \
  libbluetooth-dev \
  libcwiid-dev \
  libeigen3-dev \
  libgoogle-glog-dev \
  liblapack-dev \
  libpcl-dev \
  libspnav-dev \
  libusb-dev \
  lsb-release \
  mercurial \
  pkg-config \
  python3-dbg \
  python3-empy \
  python3-pip \
  python3-venv \
  python3-sklearn \
  software-properties-common \
  sudo \
  swig \
  wget \
  cmake-curses-gui \
  geany \
  tmux \
  iputils-ping \
  default-jre \
  iproute2 \
  zstd \
  && apt-get clean

# Install a good neovim version
RUN add-apt-repository -y ppa:neovim-ppa/stable \
  && apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y neovim

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id=1000
ENV USER=dcist
RUN deluser ubuntu && useradd -U --uid ${user_id} -ms /bin/bash $USER \
  && echo "$USER:$USER" | chpasswd \
  && adduser $USER sudo \
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Set locales
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
  locale-gen
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US.UTF-8

# enable all nvidia capabilities
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Commands below run as the developer user
USER $USER

# Remove all opencv packages installed by default
RUN sudo apt-get purge -y '*opencv*'

# Install ROS Jazzy
RUN sudo apt install software-properties-common \
  && sudo add-apt-repository universe

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')\
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && sudo apt install /tmp/ros2-apt-source.deb \
  && sudo apt update \
  && sudo apt install -y \
  ros-dev-tools

RUN sudo apt update \
  && sudo apt upgrade -y \
  && sudo apt install -y \
  ros-jazzy-desktop

# Install Clearpath Packages
RUN wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add - \
  && sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list' \
  && sudo apt update\
  && sudo apt install -y ros-jazzy-clearpath-robot

RUN /bin/sh -c 'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc'