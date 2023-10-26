ARG ROS_DISTRO=iron
FROM ros:${ROS_DISTRO} as dev
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Create Colcon workspace with the project and its deps
ENV WS=/ws
ENV PROJ_SRC=${WS}/src/naoqi_driver2
COPY . $PROJ_SRC
WORKDIR $WS/src
RUN apt install -y git python3-vcstool
RUN vcs import < $PROJ_SRC/dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR $WS
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update -y \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano libgmock-dev

# Expect to mount the source code from the host.
VOLUME $PROJ_SRC

# Every bash instance should source the entrypoint
RUN echo "source ${PROJ_SRC}/docker/entrypoint.sh" >> /root/.bashrc
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/bin/bash" ]

FROM dev as dev-prebuilt
RUN colcon build --symlink-install