# ROS2 Packages and Nodes
## Commando's
Under contruction

In deze workshop leer je een aantal basis ROS2 commando's die je in alle volgende workshops zult toepassen. Nu zullen de commando's nog volledig uigetyped zijn, later zul je ze zelf moeten kunnen toepassen. Let dus goed op wat er bebeurt(scherm-output) bij het uitvoeren van een commando.

Tip: maak gebruik van een cheatsheet, bijvoorbeeld deze van [TheConstruct](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf).


[Officieel Understanding nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

## Verkrijgen van hulp bij ROS2-commando's
Maak het complete command regel niet af 
```bash
ros2
```
of
```bash
ros2 --help
```
of
```bash
ros2 -h
```
Hulp bij een sub-commando werk net zo
```bash
ros2 pkg
```

## Verkrijgen van een lijst van packages
```bash
ros2 pkg list
```
Filter de lijst door het Linux grep opdracht te pijpen (**| grep**) Hieronder wordt gefilterd op alle packages met de naam 'node'
```bash
ros2 pkg list | grep node
```
## Starten van een ROS2-node
Een node wordt gestart met de volgende commando structuur
```
ROS2 run <package-naam> <node-naam>
```
```bash
ros2 run node_demo demo_node
```
Verkrijgen van een lijst met node die worden uitgevoerd(open hiervoor een nieuwe terminal)
```bash
ros2 node list
```
Gedetialleerde informatie over een specefieke node
```bash
ros2 node info /demo_node
```
nb. De informatie van deze node is niet belangrijk voor deze oefening

Beeindigen van een node

Schakel naar de terminal waarin de node executeert en druk op ctrl+c.

## Starten van meerdere nodes gelijktijdig
Het starten van meerdere nodes kan gebeuren roor het **ros2 launch** commando
```
ROS2 launch <package-naam> <launchfile-naam>
```
```bash
ros2 launch range_sensor welcome.launch.py
```
## Opbouw van een package weergeven

Schakel naar de juits map/directory
```bash
cd ~/ros2_industrial_ws/src/ROS2_industrial/1_basics/
```
Vraag de boomstructuur op met het tree-commando (Dit is een linux-commado)

```bash
tree node_demo/
```
n.b. als tree als commando niet wordt geaccepteerd, dan kun dit commando als volgt installeren
```bash
sudo apt install tree
```


