#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>

#include <cmath>

#if defined(MULTI_COLOR_LED)
  #include <WS2812FX.h>
  WS2812FX ws2812fxStatus = WS2812FX(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  #define RGB_BRIGHTNESS 10 // Change white brightness (max 255)
#endif


#define TURTLEBOT3_BURGER_MAX_LIN_VEL  0.22
#define TURTLEBOT3_BURGER_MAX_ANG_VEL  2.84

#define TURTLEBOT3_WAFFLE_MAX_LIN_VEL  0.26
#define TURTLEBOT3_WAFFLE_MAX_ANG_VEL  1.82
#define MAX_INPUT_VOLTAGE              2.8

#if defined(TURTLEBOT3_BURGER)
#define MAX_LIN_VEL TURTLEBOT3_BURGER_MAX_LIN_VEL
#define MAX_ANG_VEL TURTLEBOT3_BURGER_MAX_ANG_VEL
#elif defined(TURTLEBOT3_WAFFLE)
#define MAX_LIN_VEL TURTLEBOT3_WAFFLE_MAX_LIN_VEL
#define MAX_ANG_VEL TURTLEBOT3_WAFFLE_MAX_ANG_VEL
#else
#error
#endif

#define LINEAR_RESOLUTION (MAX_LIN_VEL/MAX_INPUT_VOLTAGE*2.0)// Half scale
#define ANGULAR_RESOLUTION (-MAX_ANG_VEL/MAX_INPUT_VOLTAGE*2.0)// Half scale

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




int angular_offset, linear_offset;

float round(float waarde, int decimalen) {
    float factor = std::pow(10, decimalen);
    return std::round(waarde * factor) / factor;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  float angular, linear;
  if (timer != NULL) {
    int linear, angular;

    linear = analogReadMilliVolts(LINEAR_PIN)- linear_offset;
    angular = analogReadMilliVolts(ANGULAR_PIN) - angular_offset;
    //Serial.printf("Linear = %02f, Angular = %02f\n", linear, angular);
    twist.linear.x = linear  * LINEAR_RESOLUTION * SPEED_FACTOR /1000.0;
    twist.angular.z = angular * ANGULAR_RESOLUTION * SPEED_FACTOR /1000.0;
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

#if defined(WIFI)
  WiFi.setHostname("JoystickController");
  set_microros_wifi_transports(SSID, SSID_PASSWORD, AGENT_IP_ADDRESS, (size_t)PORT);
#else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
#endif

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
  delay(2000);
  linear_offset = analogReadMilliVolts(LINEAR_PIN);
  angular_offset = analogReadMilliVolts(ANGULAR_PIN);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

#if defined(MULTI_COLOR_LED)
    ws2812fxStatus.service();
#endif
}