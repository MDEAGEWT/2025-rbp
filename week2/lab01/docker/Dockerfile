
FROM ubuntu:24.04
# change to your name and email
LABEL maintainer="kmk <kmk6061602@cbnu.ac.kr>"

# Install vim, sudo, curl
RUN apt-get update && apt-get install --quiet -y \
    ### FILL HERE ###

    ### --------- ###
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install wget, git, gosu
### FILL HERE ###

# make ros2_ws directory at /home/user

### --------- ###

# Create user as uid 1001
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# Add user to sudoers
RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

# Remove ubuntu user (from 24.04 ubuntu image has default user ubuntu, to use entrypoint.sh need to delete it)
RUN userdel -r ubuntu

# Entrypoint
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]