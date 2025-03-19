# microROS

Met microROS is het mogelijk om een communicatie kanaal tussen een embedded systeem en ROS2 op te bouwen. Dit communicatie kanaal maakt het mogelijk om dan op de embedded systeem de volgende ROS2 methodes te gebruiken:
* Topics
* Services
* Actions

Gedetaileerde informatie over MicrROS zie website: [microROS](https://micro.ros.org/)

microROS maakt gebruik van een agent welke als brug(bridge) tussen het embedded systeem en ROS2 fungeert.

Je kunt kunt de microROS-agent als volgt installeren:


:::::{card} 
::::{tab-set}

:::{tab-item} Installatie script

```bash
cd ~/ros2_industrial_ws/src/ROS2_industrial/microros/scripts
./install_microros_agent.sh
source ~/microros_ws/install/setup.bash
```
:::

:::{tab-item} Handmatig installeren

Zie: [first_application_linux](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

:::

::::

:::::

{octicon}`alert;2em;sd-text-info` Om een microROS applicatie te ontwikkelen dient in Visual Studie Code de **PlatformIO** plugin geinstalleerd.
Voor **WSL** is dat beschreven in de [Windows Subsystem for Lunix](https://avansmechatronica.github.io/WindowsSubsystemForLinuxHandleiding/documentation/WSL_Handleiding.html#platform-io)

Er zijn in het kader van deze modules al twee microROS implementaties gerealiseerd voor de volgende workshops:

* ROS2 Basics: [ranges-sensor](../../1_basics/ESP32/ultrasonic_sensor.md)

* Manipulation: [joystick](../../3_navigation/ESP32/joystick.md)

De microROS-agent kan worden gestart met het volgende commando:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev <device>
```

Afhankelijk op welke USB poort je het embedded systeem hebt aangesloten dien je
device in te vullen.
 Bijvoorbeeld;
 * /dev/ttyUSB0
 * /dev/ttyACM0

{octicon}`alert;2em;sd-text-info` Bij gebruik van WSL moet je het embedded systeem wel eerst met WSL verbinden, zie [Koppelen USB-devices aan WSL-Distributie](https://avansmechatronica.github.io/WindowsSubsystemForLinuxHandleiding/documentation/WSL_Handleiding.html#koppelen-usb-devices-aan-wsl-distributie)


 {octicon}`bell;2em;sd-text-info` Het device wordt getoond bij het uploaden/programmeren van het embedded systeem in Visual Code met de Platform IO plugin

