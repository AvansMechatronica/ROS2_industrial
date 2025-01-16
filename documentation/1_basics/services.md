# ROS2 Services
## Commando's
Under contruction

![Image](https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif)


[Officieel: Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

In deze workshop ga je de gemeten hoogte van de doos uit de topics workshop omrekenen van meters naar inches. We gebruiken hiervoor het ROS services mechanisme. Deze service is al voorbereid.
je kunt de service starten met het volgende commando:
```bash
ros2 run range_sensor metres_to_inches_server
```

Om een lijst te verkrijgen van services die actief zijn gebruik je het commando
```bash
ros2 service list
```
Je krijgt nu een list met alle services die actief zijn. Voor deze opdrachten is allen de service __/metres_to_inches__ van belang. Er zijn nog een aan services aanwezig die beginnen met __/metres_to_inches__ en gevolgd door worden door __service/....__, deze zijn voor ROS2 intern gebruik.

Elke service wordt gedefineerd door een type, Dit type kun je als volgt opvragen:
```bash
ros2 service type /metres_to_inches
```
Ook van de service kun je het protoptype opvragen:
```bash
ros2 interface proto <service_type>
```
Dit prototype is gedefineerd in een servicebeschrijving welke je kunt vinden in het volgende bestand:
**~/ros2_industrial_ws/src/ROS2_industrial/1_basics/range_sensors_interfaces/srv/ConvertMetresToInches.srv**

```bash
gedit ~/ros2_industrial_ws/src/ROS2_industrial/1_basics/range_sensors_interfaces/srv/ConvertMetresToInches.srv 
```
In deze service beschrijving vind je een opdeling in twee elen gescheiden door __"---"___:
* 1e deel: Alle parameters waarmee je de service kunt aanvragen,__request__ genoemd:
    * float64 distance_metres
* 2e deel: Parameters die de service na behandeling terug geeft, __response__ genoemd:
    * float64 distance_inches
    * bool success (deze wordt gebruikt om aan te merken dat in de service afhandeling geen fouten zijn opgetren en is optioneel)

```bash
ros2 service call /metres_to_inches range_sensors_interfaces/srv/ConvertMetresToInches "{distance_metres : 1.0}"
```
