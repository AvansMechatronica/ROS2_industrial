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


// Publisher-object voor het verzenden van sensordata via micro-ROS.
rcl_publisher_t sensor_information_publisher;
// Berichtenstructuur waarin zowel metadata als meetdata van de ultrasoonsensor staat.
range_sensors_interfaces__msg__SensorInformation sensor_information;


// Executor verwerkt callbacks (zoals de timer-callback) in de hoofdloop.
rclc_executor_t executor;
// Support-struct bevat context en init-informatie voor rclc.
rclc_support_t support;
// Standaard allocator voor geheugenbeheer binnen ROS 2 C API.
rcl_allocator_t allocator;
// Node-representatie van deze microcontroller binnen het ROS-netwerk.
rcl_node_t node;
// Timer die periodiek metingen triggert en publiceert.
rcl_timer_t timer;

// Naam van de ROS-node zoals deze zichtbaar is in het ROS-ecosysteem.
#define NODE_NAME "sensor_info_publisher"

// Macro voor "harde" foutcontrole:
// bij elke fout gaat het systeem in een veilige foutlus.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Macro voor "zachte" foutcontrole:
// fout wordt genegeerd zodat het systeem kan blijven draaien.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Houdt de huidige toestand van de status-led bij tijdens foutknipperen.
bool errorLedState = false;

// Sensorbereik in meters, gebruikt in het ROS Range-bericht.
#define MAX_RANGE  1.00
#define MIN_RANGE  0.10


// Foutafhandeling:
// - schrijft een foutmelding naar de seriële monitor
// - knippert de status-led oneindig
// - stopt daarmee effectief alle normale programmastromen
void error_loop(){
  Serial.printf("Ultrasonic Sensor\nError\nSystem halted");
  while(1){
    if(errorLedState){
        digitalWrite(STATUS_LED_PIN, HIGH);
        errorLedState = false;
    }
    else{
        digitalWrite(STATUS_LED_PIN, LOW);
        errorLedState = true;
    }
    delay(100);
  }
}

// Timer-callback die periodiek wordt aangeroepen door de executor.
// Verantwoordelijkheden:
// 1) Tijdstempel invullen
// 2) Afstand meten met HCSR04
// 3) Meting omrekenen naar meters
// 4) Bericht publiceren op topic "sensor_info"
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  // Deze callback gebruikt de last_call_time niet expliciet.
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Lees actuele systeemtijd voor ROS-header (sec + nanosec).
    struct timespec ts;
    extern int clock_gettime(clockid_t unused, struct timespec *tp);
    clock_gettime(CLOCK_REALTIME, &ts);
    sensor_information.sensor_data.header.stamp.sec = ts.tv_sec;
    sensor_information.sensor_data.header.stamp.nanosec = ts.tv_nsec;

    // Meet afstand in centimeter; index 0 bevat de eerste (en hier enige) meting.
    double* distances = HCSR04.measureDistanceCm();
    //Serial.printf("Distance: %.2f cm\n", distances[0]);
    // Converteer cm naar meter zodat het voldoet aan sensor_msgs/Range.
    sensor_information.sensor_data.range= (float)(distances[0]/100.0);

    // Publiceer bericht; bij fout geen harde stop (zachte check).
    RCSOFTCHECK(rcl_publish(&sensor_information_publisher, &sensor_information, NULL));
  }
}

void setup() {
  // Hardware-initialisatie: status-led en pinnen voor de ultrasoonsensor.
  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);

  pinMode(SR04_TRIG_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);

  // Seriële verbinding voor logging en micro-ROS transport.
  Serial.begin(115200);

  // Stel micro-ROS transport in op de seriële poort.
  set_microros_serial_transports(Serial);

  // Korte wachttijd om seriële verbinding en agent-opstart te stabiliseren.
  delay(2000);

  // Start de HCSR04 driver met de gekozen trigger- en echo-pin.
  HCSR04.begin(SR04_TRIG_PIN, SR04_ECHO_PIN);

  // Verkrijg standaard allocator voor alle rcl/rclc initialisaties.
  allocator = rcl_get_default_allocator();

  // Initialiseer rclc support (context + basisinfrastructuur).
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Maak ROS-node aan met vaste naam.
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Vul statische metadata van het custom bericht in.
  // Deze waarden blijven gelijk voor alle publicaties.
  rosidl_runtime_c__String__assign(&sensor_information.sensor_data.header.frame_id, "distance_sensor_frame");
  rosidl_runtime_c__String__assign(&sensor_information.maker_name, "Avans");
  sensor_information.part_number = 20241102;

  // Vul sensor-specifieke eigenschappen in volgens sensor_msgs/Range.
  sensor_information.sensor_data.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  sensor_information.sensor_data.field_of_view = 0.5; // Field of view of the sensor in rad.
  sensor_information.sensor_data.min_range = (float)MIN_RANGE; // Minimum distance range of the sensor in m.
  sensor_information.sensor_data.max_range = MAX_RANGE; // Maximum distance range of the sensor in m.


  // Maak publisher voor het custom bericht op topic "sensor_info".
  RCCHECK(rclc_publisher_init_default(
    &sensor_information_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(range_sensors_interfaces, msg, SensorInformation),
    "sensor_info"));


  // Configureer periodieke timer (1000 ms = 1 Hz).
  // Elke tick activeert timer_callback() voor meten + publiceren.
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialiseer executor en koppel de timer eraan.
  // De executor wordt in loop() periodiek "gespind".
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Led uit: setup afgerond, systeem draait normaal.
  digitalWrite(STATUS_LED_PIN, LOW);

}

void loop() {
  // Kleine vertraging om CPU-belasting te beperken.
  delay(100);
  // Verwerk pending callbacks (zoals timer events) gedurende 100 ms.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}