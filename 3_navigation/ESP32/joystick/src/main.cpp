#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>

#include <cmath>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#if defined(MULTI_COLOR_LED)
  #include <WS2812FX.h>
  WS2812FX ws2812fxStatus = WS2812FX(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  #define RGB_BRIGHTNESS 10 // Change white brightness (max 255)
#endif

rcl_publisher_t twist_publisher;
geometry_msgs__msg__Twist twist;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define NODE_NAME "twist_publisher"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

bool errorLedState = false;


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
#define SPEED_FACTOR      0.5
//#define ANGULAR_PIN 35
//#define LINEAR_PIN  34
#define LINEAR_RESOLUTION (0.26/4096.0*2.0)// Half scale
#define LINEAR_OFFSET     (4096/2)
#define ANGULAR_RESOLUTION (-1.28/4096.0*2.0)// Half scale
#define ANGULAR_OFFSET     (4096/2)

float angular_offset, linear_offset;

float round(float waarde, int decimalen) {
    float factor = std::pow(10, decimalen);
    return std::round(waarde * factor) / factor;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  float angular, linear;
  if (timer != NULL) {
    float angular, linear;

    angular = (analogRead(ANGULAR_PIN) - ANGULAR_OFFSET) * ANGULAR_RESOLUTION;
    angular = round(angular, 2) - angular_offset;
    linear = (analogRead(LINEAR_PIN) - LINEAR_OFFSET) * LINEAR_RESOLUTION;
    linear = round(linear, 2) - linear_offset;
    //Serial.printf("Linear = %02f, Angular = %02f\n", linear, angular);
    twist.linear.x = linear  * SPEED_FACTOR;
    twist.angular.z = angular * SPEED_FACTOR;
    RCSOFTCHECK(rcl_publish(&twist_publisher, &twist, NULL));
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
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));


  // create twist_publisher
  RCCHECK(rclc_publisher_init_default(
    &twist_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));


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

  // Callibrate zerro offset
  angular_offset = (analogRead(ANGULAR_PIN) - ANGULAR_OFFSET) * ANGULAR_RESOLUTION;
  angular_offset = round(angular_offset, 2);
  linear_offset = (analogRead(LINEAR_PIN) - LINEAR_OFFSET) * LINEAR_RESOLUTION;
  linear_offset = round(linear_offset, 2);
}




void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

#if defined(MULTI_COLOR_LED)
    ws2812fxStatus.service();
#endif
}