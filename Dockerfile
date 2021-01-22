#-----------------
# Stage 1 - build
#-----------------

FROM ros:foxy AS builder

COPY . /root/rmf/src

SHELL ["bash", "-c"]

# add gazebo/ignition repo
RUN apt-get update && apt-get install -y curl wget \
  && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# install ros dependencies
RUN cd /root/rmf && apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -yr

# other dependencies
RUN apt-get update && apt-get install -y \
  # needed by traffic editor
  libignition-common3-dev libignition-plugin-dev

# build rmf
RUN . /opt/ros/foxy/setup.bash && cd /root/rmf && \
  colcon build --merge-install --install-base /opt/rmf --cmake-args -DCMAKE_BUILD_TYPE=Release

#----------
# Stage 2
#----------

# TODO: rosdep doesn't support installing only exec dependencies (https://github.com/ros-infrastructure/rosdep/pull/727)
#   When the PR is merged, we can do a multi-stage build and include only whats needed at runtime.
# FROM ros:foxy
# COPY --from=0 /root/rmf/install /opt/rmf
# RUN rosdep ...
# COPY --from=builder /root/rmf/install /opt/rmf

# cleanup
RUN rm -rf /root/rmf /var/lib/apt/lists/*

ADD docker/rmf_entrypoint.bash /
ENTRYPOINT ["/rmf_entrypoint.bash"]
CMD ["bash"]
