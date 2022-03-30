#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rdata_msgs/msg/log_f64.h>
#include <rdata_msgs/srv/create_logger.h>
#include <micro_ros_utilities/string_utilities.h>

#if !defined(TARGET_STM32F4) && !defined(ARDUINO_TEENSY41) && !defined(TARGET_PORTENTA_H7_M7)
#error This example is only available for Arduino Portenta, Arduino Teensy41 and STM32F4
#endif

rcl_publisher_t publisher;
rdata_msgs__msg__LogF64 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}



void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    //msg.data++;
  }
}

void setup() {
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  IPAddress arduino_ip(192, 168, 2, 2);
  IPAddress agent_ip(192, 168, 2, 1);
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 8080);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "uros_node", "uros", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rdata_msgs, msg, LogF64),
    "data"));

  msg.measurment = micro_ros_string_utilities_set(msg.measurment, "measurement");
  msg.sensor = micro_ros_string_utilities_set(msg.sensor, "sensor");
  msg.value = 0.0;
}

void loop() {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.value = msg.value + 1.0;
    delay(100);
}
