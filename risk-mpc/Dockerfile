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
ENV LANGUAGE=n_US:en

# enable all nvidia capabilities
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Commands below run as the developer user
USER $USER

# Remove all opencv packages installed by default
RUN sudo apt-get purge -y '*opencv*'

# Install ROS Jazzy
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && sudo apt-get update \
  && sudo apt-get install -y \
  ros-dev-tools \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-desktop

RUN sudo rosdep init \
  && sudo apt-get clean

RUN rosdep update

# Install ROS dependencies
RUN sudo apt-get update \
  && sudo apt-get install -y \
  ros-jazzy-nmea-msgs \
  ros-jazzy-nmea-navsat-driver \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-twist-stamper \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gtsam \
  ros-jazzy-pcl-ros \
  ros-jazzy-octomap \
  ros-jazzy-octomap-msgs \
  ros-jazzy-vision-msgs \
  ros-jazzy-mavros \
  ros-jazzy-mavros-msgs \
  ros-jazzy-imu-tools \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rmw-fastrtps-cpp

# Install geographiclib_datasets required by mavros
# RUN cd /opt/ros/jazzy/lib/mavros/ && sudo ./install_geographiclib_datasets.sh

# Install pytorch for the appropriate architecture
# RUN if [ "$BARE_OR_NVDA" = "nvda" ] ; then \
#   if [ "$ARCHITECTURE" = "x86_64" ] ; then \
#     echo "nvda - x86_64"; sudo pip3 install --break-system-packages torch==2.4.0 torchvision==0.19.0 torchaudio==2.4.0 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cpu ; \
#     else \
#     echo "nvda - arm64" ; echo "Skipping pytorch installation for Jetson"; \
#   fi; \
# else \
#   if [ "$ARCHITECTURE" = "amd64" ] ; then \
#     echo "bare - x86_64"; sudo pip3 install --break-system-packages torch==2.4.0 torchvision==0.19.0 torchaudio==2.4.0 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cpu ; \
#   else \
#     echo "Architecture not supported"; exit 1; \
#   fi; \
# fi;

# Install dependencies for MOCHA
RUN sudo apt-get install -y \
  python3-zmq \
  python3-rospkg \
  python3-defusedxml \
  python3-utm

# Install all dependencies for ROFL. Based on the ROFL build action
# https://github.com/versatran01/rofl-beta/blob/main/.github/workflows/build.yaml
# RUN mkdir -p ~/cpp_deps
# RUN cd ~/cpp_deps \
#   && git clone --depth 1 --branch v0.6.0 https://github.com/google/glog.git \
#   && cd glog \
#   && cmake -S . -B build -G "Unix Makefiles" -DCMAKE_CXX_STANDARD=17 \
#   && cmake --build build \
#   && sudo cmake --build build --target install
# RUN cd ~/cpp_deps \
#   && git clone --depth 1 --branch 8.1.0 https://github.com/fmtlib/fmt.git \
#   && cd fmt && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DCMAKE_CXX_STANDARD=17 -DFMT_TEST=False \
#   && sudo make install
# RUN cd ~/cpp_deps \
#   && git clone --depth 1 --branch 20220623.0 https://github.com/abseil/abseil-cpp.git \
#   && cd abseil-cpp \
#   && mkdir build \
#   && cd build \
#   && cmake -DABSL_BUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. \
#   && sudo sudo cmake --build . --target install
# RUN cd ~/cpp_deps \
#   && git clone https://github.com/strasdat/Sophus.git \
#   && cd Sophus \
#   && git checkout 785fef3 \
#   && mkdir build \
#   && cd build \
#   && cmake -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=17 .. \
#   && sudo make install -j8
# RUN cd ~/cpp_deps \
#   && git clone https://github.com/google/benchmark.git \
#   && cd benchmark && cmake -E make_directory "build" \
#   && cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../ \
#   && sudo cmake --build "build" --config Release --target install
# RUN cd ~/cpp_deps \
#   && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
#   && cd Pangolin \
#   && git checkout v0.9.1 \
#   && sudo apt install -y $(./scripts/install_prerequisites.sh required --list) \
#   && cmake -B build \
#   && sudo cmake --build build --config Release --target install

# Build Ian's ORBSLAM3 version with a few patches (thanks Varun) - Currently has build issues
# RUN mkdir ~/Repos && cd ~/Repos \
#   && git clone https://github.com/iandouglas96/ORB_SLAM3.git && cd ORB_SLAM3 \
#   && rm -rf Thirdparty/Sophus \
#   && sed -i '/${PROJECT_SOURCE_DIR}\/Thirdparty\/Sophus/c\#${PROJECT_SOURCE_DIR}\/Thirdparty\/Sophus' CMakeLists.txt \
#   && sed -i '/#include "Thirdparty\/Sophus\/sophus\/geometry.hpp"/c\#include "sophus\/geometry.hpp"' include/Frame.h \
#   && sed -i '/#include "Thirdparty\/Sophus\/sophus\/geometry.hpp"/c\#include "sophus\/geometry.hpp"' include/Converter.h \
#   && sed -i '/#include "Thirdparty\/Sophus\/sophus\/sim3.hpp"/c\#include "sophus\/sim3.hpp"' include/Converter.h \
#   && sed -i '/find_package(OpenCV 3)/c\find_package(OpenCV 4)' CMakeLists.txt \
#   && sed -i '/CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)/c\CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)' CMakeLists.txt \
#   && sed -i '/if(COMPILER_SUPPORTS_CXX11)/c\if(COMPILER_SUPPORTS_CXX14)' CMakeLists.txt \
#   && sed -i '/set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")/c\set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")' CMakeLists.txt \
#   && sed -i '23 i #include<iostream>' include/ImuTypes.h \
#   && sed -i '23 i #include<iostream>' include/GeometricTools.h \
#   && sed -i '126 i -lfmt' CMakeLists.txt \
#   && chmod +x build.sh && ./build.sh
# RUN cd ~/Repos/ORB_SLAM3/Vocabulary/ && tar xf ORBvoc.txt.tar.gz

# Clone all the gazebo models so that you don't download them every time
# docker is run
# RUN mkdir -p /home/$USER/.gazebo && git clone https://github.com/osrf/gazebo_models /home/$USER/.gazebo/models

# When running a container start in the developer's home folder
WORKDIR /home/$USER

# Configure .bashrc
RUN echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

# Source ROS repositories
RUN /bin/sh -c 'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc' 

# Fix permissions of mounted folders
RUN /bin/sh -c 'echo sudo chown $USER:$USER ~/dcist_ws >> ~/.bashrc'
# RUN /bin/sh -c 'echo sudo chown $USER:$USER ~/data >> ~/.bashrc'
RUN /bin/sh -c 'echo sudo chown $USER:$USER ~/.ros >> ~/.bashrc'

# install micro-ROS setup, create a micro-ROS agent
RUN cd /home/dcist \
  && mkdir microros_ws \
  && cd microros_ws \
  && git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
  && sudo apt update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -y \
  && /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build  && \
    source install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh" 
RUN /bin/sh -c 'echo "source /home/dcist/microros_ws/install/setup.bash" >> ~/.bashrc'

# install Livox-SDK2 for Ubuntu 24.04
RUN cd /home/dcist \
  && git clone https://github.com/StarpathRobotics/Livox-SDK2.git \
  && cd Livox-SDK2 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j $(nproc) \
  && sudo make install

# Copy the dcist_ws environment
COPY --chown=$USER:$USER ../ws /home/$USER/dcist_ws

# clone livox_ros_driver2, build, and COLCON_IGNORE it
RUN cd /home/dcist/dcist_ws/src \
  && /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2 && \
    cd ws_livox/src/livox_ros_driver2 && \
    ./build.sh humble && \
    cd /home/dcist/dcist_ws/src/ws_livox && \
    touch COLCON_IGNORE"

# download and install the ZED SDK
RUN cd /home/dcist \
  && wget -O zed_sdk.run https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu24?_gl=1*1x6bx14*_gcl_au*MTUzMjE4OTEyOS4xNzQwMTczNjA4 \
  && chmod +x zed_sdk.run \
  && ./zed_sdk.run -- silent skip_cuda skip_od_module\
  && rm zed_sdk.run

# install the ZED ROS2 wrapper
# install the ZED ROS2 wrapper
RUN cd /home/dcist/dcist_ws/src/ \
  && git clone --recursive https://github.com/frankgon1627/zed-ros2-wrapper.git

# install and build casadi from source
RUN cd /home/dcist \
  && git clone https://github.com/casadi/casadi.git -b main casadi \
  && cd casadi \
  && git checkout 6704bbe \
  && mkdir build \
  && cd build \
  && cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON ..\
  && make -j $(nproc) \
  && sudo make install

# Temporary hack solution to fix ROS2 installation issues
RUN cd /etc/apt/sources.list.d \
  && sudo rm -f ros2.list \
  && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')\
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

RUN sudo apt install /tmp/ros2-apt-source.deb \
  && cd /home/dcist/dcist_ws \
  && /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    source /home/dcist/microros_ws/install/setup.bash && \
    source /home/dcist/dcist_ws/src/ws_livox/install/setup.bash && \
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro=jazzy -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'
    
RUN /bin/sh -c 'echo "source /home/dcist/dcist_ws/src/ws_livox/install/setup.bash" >> ~/.bashrc'
RUN /bin/sh -c 'echo "source /home/dcist/dcist_ws/install/setup.bash" >> ~/.bashrc'
# RUN /bin/sh -c 'echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc' 
# RUN /bin/sh -c 'echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc' 

