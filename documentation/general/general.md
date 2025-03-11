# Algemene informatie

## Pre-requistions ROS2-Humble
Installeer ROS Humble, kies uit 1 van de twee volgende mogelijkheden:
* WSL Distributie onder Windows(voorkeur voor studenten M)
* Native Ubuntu


````{tab-set}

```{tab-item} WSL Distributie
:sync: tabgroup

Zie voor installatie: [Windows Subsystem for Linux Handleiding](https://avansmechatronica.github.io/WindowsSubsystemForLinuxHandleiding/)
* Volg instrucies voor ROS2 Humble
```

```{tab-item} Native Ubuntu
:sync: tabgroup

*Opmerking: je dient eerst Ubuntu 22 geinstallerd te hebben!!!*

Volg de instructies voor het [installeren ROS-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Kies: ros-humble-desktop

```
````


## Cloning de ROS2 Industrial workspace
```bash
mkdir -p ~/ros2_industrial_ws/src
cd ~/ros2_industrial_ws/src
git clone https://github.com/AvansMechatronica/ROS2_industrial.git
```

## Installeren van de benodigde ROS2 packages en software
```bash
cd ~/ros2_industrial_ws/src/ROS2_industrial/install
./install.bash
```

## Bouwen van de ROS2 Industrial workspace
```bash
cd ~/ros2_industrial_ws/
colcon build --symlink-install
source install/setup.bash
echo "source ~/ros2_industrial_ws/install/setup.bash" >> ~/.bashrc
```
Opmerking: *Gebruik de laatse regel slechts 1 maal.*

## Aantekening voor windows gebruikers
Opmerking: *Alleen als je geen gebruik maakt van de door Avans gemaakte WSL-Humble distributie*

Je kunt de WSL Ubuntu-22.04 distributie uit de Microsoft Store gebruiken. Gebruik als ontwikkelomgeving [Visual Studio Code] (https://code.visualstudio.com/download). Wijzig de installatie en voeg de WSL-plugin toe aan Visual Studio Code. Open de distributie met <F1>WSL: Connect to WSL. Vergeet niet ROS-Humble te installeren in de distributie. Cloon deze repositry naar de WSL distributie.

## Inleveren van opdrachten
Als je een opdracht dient in te leveren voor je opleiding dan vind je dat terug in de Elekronische Leeromgeving van je opleiding (bijvoorbeeld Brightspace). Zorg ervoor dat je bij elke opdracht de volgende informatie in het bestand invult:
* Naam Student
* Studentnummer
* Datum

Je gaat dan akkoord met onderstaande verklaring:
~~~
Door het inleveren van dit bestand verklaar ik dat 
ik deze opdracht zelfstandig heb uitgevoerd en
dat ik geen code van anderen heb gebruikt.
Tevens ga ik akkoord met de beoordeling van deze opdracht.
~~~


## Verantwoording

Deze workshop is geinspireerd op [hello real world ros robot operating system](https://ocw.tudelft.nl/courses/hello-real-world-ros-robot-operating-system/) van de Technische Universiteit Delft/Nederland(TUD)

