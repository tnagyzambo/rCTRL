#!/usr/bin/env bash

# Start influxd process and send to background
influxd &

# Wait for influxd to start
sleep 15s

# Inital setup of influx
# REFERENCE: https://docs.influxdata.com/influxdb/v2.0/reference/cli/influx/setup/#flags
influx setup -u ${INFLUX_USER} -p ${INFLUX_PASSWORD} -o ${INFLUX_ORG} -b ${INFLUX_BUCKET} -r ${INFLUX_RETENTION} -f

# Regex match for ros user's API token and set as enviroment variable for use with ROS
# Add as enviroment variable for all shells
# REFERENCE: https://regexr.com/
# '([^\s]*)(?=\s+\bros(?![^\s])\b)'
export TOKEN_FILE=/ros-influx-api-token.tokenfile
touch ${TOKEN_FILE}
echo $(influx auth list | grep -oP '([^\s]*)(?=\s+\bros(?![^\s])\b)') >> ${TOKEN_FILE}

rm /influx-setup.sh

echo ""
echo "Press any key to close container..."
read -rsp -n1 key
