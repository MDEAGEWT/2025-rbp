FROM ubuntu:24.04
# change to your name and email
LABEL maintainer="kmk <kmk6061602@cbnu.ac.kr>"

RUN apt-get update \
	&& apt-get install --quiet -y \
        sudo \
        gosu \
		curl \
        dpkg \
        lsb-release \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install ROS2 Jazzy
ENV ROS_DISTRO=jazzy

# setup ros2 keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# setup sources.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ros2 desktop
RUN apt-get update \
	&& apt-get install --quiet --no-install-recommends -y \
		ros-$ROS_DISTRO-desktop \
		ros-$ROS_DISTRO-launch-testing-ament-cmake \
		ros-$ROS_DISTRO-ros2bag \
		ros-$ROS_DISTRO-rosidl-generator-dds-idl \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN apt-get update \ 
	&& apt-get install --quiet --no-install-recommends -y \
		python3-colcon-common-extensions \
		ros-dev-tools \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create user as uid 1001
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# Add user to sudoers
RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

# Remove ubuntu user (from 24.04 ubuntu image has default user ubuntu, to use entrypoint.sh need to delete it)
RUN userdel -r ubuntu

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/user/.bashrc \
    && chown user:user /home/user/.bashrc

COPY ./ros2_ws /home/user/ros2_ws
RUN chown -R user:user /home/user/ros2_ws

# Entrypoint
COPY ./docker/scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]