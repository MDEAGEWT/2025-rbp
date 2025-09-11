
FROM ubuntu:24.04
# change to your name and email
LABEL maintainer="kmk <kmk6061602@cbnu.ac.kr>"

# Install vim, sudo, curl
RUN apt-get update && apt-get install --quiet -y \
    vim \
    sudo \
    curl \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install wget, git, gosu
### FILL HERE ###
RUN apt-get update && apt-get install --quiet -y \
    wget \
    git \
    gosu \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# make ros2_ws directory at /home/user
RUN mkdir -p /home/user/ros2_ws
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

RUN chown -R user:user /home/user/ros2_ws

# Add user to sudoers
RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

# Remove ubuntu user (from 24.04 ubuntu image has default user ubuntu, to use entrypoint.sh need to delete it)
RUN userdel -r ubuntu

# Entrypoint
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]