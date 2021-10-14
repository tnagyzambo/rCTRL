# rocketDATA

**rocketDATA** is a **ROS2** package designed to write all information that it receives to an **InfluxDB 2.0** instance for easy and robust data visualization and logging. The intended use is for Raspberry Pi based flight computers in student rocketry but also has a wide range of alternative applications.

# Setup

**InfluxDB** must be running before the **rocketDATA** package is started. **rocketDATA** looks for a `credentials.toml`  file in `/rocketDATA/influx/` of the host system on start to supply the necessary API credentials to write to **InfluxDB**.

A **Docker** test container is provided that automates the creation of a valid running environment for **rocketDATA**. It is recommended that this test container is adapted to generate the production environment as well. Manually creating the running environment for **rocketDATA** is *NOT* recommended as it is somewhat involved and steps can easily be missed.

# Use

Automated start instructions can be found under `.vscode/tasks.json` within this repo. It is recommended to use the VS Code extension **Task Explorer** to launch these tasks within the test container environment. These tasks can be easily translated to the production environment.

**rocketDATA** will listen to the following **ROS2** topics:
- `rocketDATA_LogBool`
- `rocketDATA_LogFloat64`
- `rocketDATA_LogInt64`
- `rocketDATA_LogString`
- `rocketDATA_LogUint64`

Upon hearing messages on these topics, the package will create a POST request to **InfluxDB** to log the data.

The **ROS2** package **rocketDATA Virtual Sensor** is included with this repository to provide dummy data to aid in any testing or debugging as well as to provide a reference for future publishing **ROS2** nodes.

# Visulization

You can log into the running **InfluxDB** instance on the host machine via `localhost:8086`. It is recommended to use a **Grafana** instance connected to **InfluxDB** instead.