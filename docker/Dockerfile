# global args
ARG FROM_IMAGE=osrf/ros:melodic-desktop
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS updated

RUN apt update -y && apt upgrade -y \
    && apt autoremove -y && apt autoclean -y

FROM updated AS cacher

ARG SSH_PRIVATE_KEY
# add credentials on build
RUN mkdir -p /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa
RUN chmod 400 /root/.ssh/id_rsa

# make sure your domain is accepted
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

ARG CACHBUST=""

# clone workspace source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./pckgs.rosinstall ../
RUN apt update && apt install git -y\
    && rm -rf /var/lib/apt/lists/*
RUN wstool init . ../pckgs.rosinstall 
RUN wstool update -t . 

RUN git clone git@github.com:Amin173/local_sensing_intest.git

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "CATKIN_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# template build container for multi-stage building
FROM updated AS builder_temp

COPY ./config.sh /

# install catkin tools
RUN apt update \
    && apt install ros-$ROS_DISTRO-catkin python-catkin-tools python-osrf-pycommon pip pip3-y \
    && bash /config.sh
    && rm -rf /var/lib/apt/lists/*

# final build container
FROM builder_temp AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN rosdep update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src\
      --ignore-src \
      --rosdistro $ROS_DISTRO\
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build 

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

CMD [ "bash"]