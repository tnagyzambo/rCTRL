FROM ubuntu:20.04 AS base

# Fix locale issues
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy locales
RUN locale-gen en_US.UTF-8

# Create non-root user
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN addgroup --gid $GROUP_ID ros
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID ros

# Install compile tools
RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy clang-12 python3-pip
ENV CC=/bin/clang-12
ENV CXX=/bin/clang++-12

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

# Install dev dependencies
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy wget libssl-dev

# Install libcurl dev dependencies
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy libcurl4-openssl-dev

# Install toml++ dev dependencies
ARG TOML11_RELEASE=2.5.0
RUN wget https://github.com/marzer/tomlplusplus/archive/refs/tags/v${TOML11_RELEASE}.tar.gz
RUN tar -xf v${TOML11_RELEASE}.tar.gz -C /lib
RUN rm v${TOML11_RELEASE}.tar.gz
RUN mkdir /lib/tomlplusplus-${TOML11_RELEASE}/build
RUN cd /lib/tomlplusplus-${TOML11_RELEASE} && cmake -S . -B build
RUN cd /lib/tomlplusplus-${TOML11_RELEASE} && cmake --build build
RUN cd /lib/tomlplusplus-${TOML11_RELEASE} && cmake --install build

# Install rosbridge
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -qy ros-galactic-rosbridge-suite

# Pull in ROS2 application dependancies
FROM base AS depends

# influxdb
# REFERENCE: https://docs.influxdata.com/influxdb/v2.0/install/?t=Linux
ARG INFLUX_RELEASE=influxdb2-2.0.7-arm64.deb
RUN wget https://dl.influxdata.com/influxdb/releases/${INFLUX_RELEASE}
RUN dpkg -i ${INFLUX_RELEASE}
RUN rm ${INFLUX_RELEASE}
# RUN service influxdb start

# Add influx credentials file
USER ros
RUN mkdir /home/ros/rdata
RUN mkdir /home/ros/rdata/influx
COPY resources/influx/credentials.toml /home/ros/rdata/influx/credentials.toml

# Add influx configuration file
COPY resources/influx/config.toml /home/ros/rdata/influx/config.toml
ENV INFLUXD_CONFIG_PATH = "/home/ros/rdata/influx"

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

# Rust
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
RUN chmod -R a+w $RUSTUP_HOME $CARGO_HOME;

# Rust/Yew dependencies
USER ros
RUN rustup target add wasm32-unknown-unknown
RUN cargo install trunk
