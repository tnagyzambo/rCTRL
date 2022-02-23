FROM ubuntu:20.04 AS base

# Create non-root user
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN addgroup --gid $GROUP_ID ros
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID ros

# Install compile tools
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy clang-12 clang-tidy gdb gdbserver python3-pip
ENV CC=/bin/clang-12
ENV CXX=/bin/clang++-12

# Install locales
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy locales

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8  

# Install ROS2
USER root
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy curl gnupg lsb-release
RUN curl -SSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Setup ROS2 enviroment
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy ros-galactic-ros-base python3-colcon-common-extensions
RUN echo "source /opt/ros/galactic/setup.bash" >> /home/ros/.bashrc

# Pull in ROS2 application dependancies
FROM base AS depends

# influxdb
# REFERENCE: https://docs.influxdata.com/influxdb/v2.0/install/?t=Linux
ARG INFLUX_RELEASE=influxdb2-2.0.7-arm64.deb
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy wget
RUN wget https://dl.influxdata.com/influxdb/releases/${INFLUX_RELEASE}
RUN dpkg -i ${INFLUX_RELEASE}
RUN rm ${INFLUX_RELEASE}
# RUN service influxdb start

# Add influx credentials file
USER ros
RUN mkdir /home/ros/rocketDATA
RUN mkdir /home/ros/rocketDATA/influx
COPY resources/influx/credentials.toml /home/ros/rocketDATA/influx/credentials.toml

# Add influx configuration file
COPY resources/influx/config.toml /home/ros/rocketDATA/influx/config.toml
ENV INFLUXD_CONFIG_PATH = "/home/ros/rocketDATA/influx"

# TLS
# REFERENCE: https://docs.influxdata.com/influxdb/v2.0/security/enable-tls/
# RUN openssl req -x509 -nodes -newkey rsa:2048 \
#     -keyout /etc/ssl/influxdb-selfsigned.key \
#     -out /etc/ssl/influxdb-selfsigned.crt
# RUN chmod 644 /etc/ssl/influxdb-selfsigned.crt
# RUN chmod 600 /etc/ssl/influxdb-selfsigned.key

# Bring in entrypoint script from local machine
# Make sure script is executable with chmod -x
COPY resources/setup.sh /home/ros/setup.sh
USER root
RUN chmod +x /home/ros/setup.sh