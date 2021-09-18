FROM ubuntu:20.04 AS base

# Create non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

# Fix locales
# RUN apt update 
# RUN DEBIAN_FRONTEND=noninteractive apt install -qy locales
# RUN locale-gen en_US en_US.UTF-8
# RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# RUN export LANG=en_US.UTF-8

# Install ROS2
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy curl gnupg lsb-release
RUN curl -SSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Setup ROS2 enviroment
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy ros-galactic-ros-base
RUN echo "source /opt/ros/galactic/setup.bash" >> /home/$USERNAME/.bashrc

# Pull in ROS2 application dependancies
FROM base AS depends

ENV INFLUX_USER=ros
ENV INFLUX_PASSWORD=rosinflux
ENV INFLUX_ORG=ros
ENV INFLUX_BUCKET=default
ENV INFLUX_RETENTION=0

# influxdb
# REFERENCE: https://docs.influxdata.com/influxdb/v2.0/install/?t=Linux
ARG INFLUX_RELEASE=influxdb2-2.0.7-arm64.deb
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy wget
RUN wget https://dl.influxdata.com/influxdb/releases/${INFLUX_RELEASE}
RUN dpkg -i ${INFLUX_RELEASE}
RUN rm ${INFLUX_RELEASE}
RUN service influxdb start

# Add influx configuration file
RUN mkdir /influx
COPY .testcontainer/resources/influx-config.toml /influx/config.toml
ENV INFLUXD_CONFIG_PATH=/influx

# TLS
# REFERENCE: https://docs.influxdata.com/influxdb/v2.0/security/enable-tls/
# RUN openssl req -x509 -nodes -newkey rsa:2048 \
#     -keyout /etc/ssl/influxdb-selfsigned.key \
#     -out /etc/ssl/influxdb-selfsigned.crt
# RUN chmod 644 /etc/ssl/influxdb-selfsigned.crt
# RUN chmod 600 /etc/ssl/influxdb-selfsigned.key


# Bring in entrypoint script from local machine
# Make sure script is executable with chmod -x
COPY .testcontainer/resources/influx-setup.sh /influx-setup.sh
RUN chmod +x /influx-setup.sh

ENTRYPOINT ["/influx-setup.sh"]

# docker build . -f .testcontainer/Dockerfile -t testcontainer

# docker run --rm -v ${PWD}/install:/install -i --name testcontainer testcontainer:latest

# docker exec -it testcontainer /bin/bash
