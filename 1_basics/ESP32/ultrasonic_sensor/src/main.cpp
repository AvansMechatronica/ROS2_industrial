#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/range.h>
#include <range_sensors_interfaces/msg/sensor_information.h>

#include <HCSR04.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#if defined(MULTI_COLOR_LED)
  #include <WS2812FX.h>
  WS2812FX ws2812fxStatus = WS2812FX(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  #define RGB_BRIGHTNESS 10 // Change white brightness (max 255)
#endif

rcl_publisher_t sensor_information_publisher;
range_sensors_interfaces__msg__SensorInformation sensor_information;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define NODE_NAME "sensor_info_publisher"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

bool errorLedState = false;
//#define STATUS_LED_PIN    2 //LED_BUILTIN //8
#define MAX_RANGE  1.00
#define MIN_RANGE  0.10
//#define SR04_TRIG_PIN   32
//#define SR04_ECHO_PIN   33

void error_loop(){
  Serial.printf("Ultrasonic Sensor\nError\nSystem halted");
  while(1){
      

#if defined(MULTI_COLOR_LED)
        ws2812fxStatus.service();
#endif
        if(errorLedState){
#if defined(MULTI_COLOR_LED)
            ws2812fxStatus.setColor(0,0,0);
#else
#if defined(STATUS_LED_PIN)
            digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#endif
            errorLedState = false;
        }
        else{
            //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);
#if defined(MULTI_COLOR_LED)
            ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
#else
#if defined(STATUS_LED_PIN)
            digitalWrite(STATUS_LED_PIN, LOW);
#endif
#endif
            errorLedState = true;
        }
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    struct timespec ts;
    extern int clock_gettime(clockid_t unused, struct timespec *tp);
    clock_gettime(CLOCK_REALTIME, &ts);
    sensor_information.sensor_data.header.stamp.sec = ts.tv_sec;
    sensor_information.sensor_data.header.stamp.nanosec = ts.tv_nsec;

    //double* distances = 0;// HCSR04.measureDistanceCm();
    double* distances = HCSR04.measureDistanceCm();
    sensor_information.sensor_data.range= (float)(distances[0]/100.0);

    RCSOFTCHECK(rcl_publish(&sensor_information_publisher, &sensor_information, NULL));
  }
}

void setup() {
  // Configure serial transport
#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.init();
  ws2812fxStatus.setMode(FX_MODE_STATIC);
  ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
  ws2812fxStatus.setBrightness(100);
  ws2812fxStatus.setSpeed(200);
  ws2812fxStatus.start();
  ws2812fxStatus.service();
#else
#if defined(STATUS_LED_PIN)
  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#endif


  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  ws2812fxStatus.service();
#endif



  HCSR04.begin(SR04_TRIG_PIN, SR04_ECHO_PIN);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // setup static values for topic
  rosidl_runtime_c__String__assign(&sensor_information.sensor_data.header.frame_id, "distance_sensor_frame");
  rosidl_runtime_c__String__assign(&sensor_information.maker_name, "Avans");
  sensor_information.part_number = 20241102;

  // Fill in the sensor data information.
  sensor_information.sensor_data.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  sensor_information.sensor_data.field_of_view = 0.5; // Field of view of the sensor in rad.
  sensor_information.sensor_data.min_range = (float)MIN_RANGE; // Minimum distance range of the sensor in m.
  sensor_information.sensor_data.max_range = MAX_RANGE; // Maximum distance range of the sensor in m.


  // create sensor_information_publisher
  RCCHECK(rclc_publisher_init_default(
    &sensor_information_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(range_sensors_interfaces, msg, SensorInformation),
    "sensor_info"));


  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.setColor(0, RGB_BRIGHTNESS,0);
#else
#if defined(STATUS_LED_PIN)
  digitalWrite(STATUS_LED_PIN, LOW);
#endif
#endif

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

#if defined(MULTI_COLOR_LED)
    ws2812fxStatus.service();
#endif
}