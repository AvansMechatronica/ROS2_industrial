# ROS2 urdf


In deze workshop leer je een aantal technieken om zelf realistische simulatie omgeving te bouwen.

## Opdracht 1
Plaats een bin(bak) op de hieronder aangegeven plaats.

::::{grid} 2
:::{grid-item-card} 
![Image](./images/assignment1_arrow_perspective.jpg)
:::
:::{grid-item-card}
![Image](./images/assignment1_arrow_top_view.jpg)
:::
::::


Bewerk daartoe het *"assignment1.urdf.xacro"* bestand in de package urdf_basics van 2_urdf directory.
Voeg je urdf-xml code toe achter de regel:

```
<!-- Add your solution to assignment 1 here -->
```

Start assignment 1
```bash
ros2 launch urdf_basics visualize_assignment1.launch.py
```

## Opdracht 2
In deze opdracht ga je een nieuw object toevoegen aan de fabriek: een groene bol.

De bol moet aan de andere kant van de transportband worden geplaatst, onder de trap aan het uiteinde van de fabriek.

Raadpleeg de volgende illustraties. De rode pijl geeft aan waar je de bol moet plaatsen.
::::{grid} 2
:::{grid-item-card} 
![Image](./images/assignment2_arrow_perspective.jpg)
:::
:::{grid-item-card}
![Image](./images/assignment2_arrow_top_view.jpg)
:::
::::



Bewerk daartoe het *"assignment2.urdf.xacro"* bestand in de package urdf_basics van 2_urdf directory.
Voeg je urdf-xml code toe achter de regel:
```
<!-- Add your solution to assignment 2 here -->
```

Start assignment 2
```bash
ros2 launch urdf_basics visualize_assignment2.launch.py
```
## Opdracht 3
In deze opdracht dien je Robot 2 (de *uFactory xArm6*) te vervangen door een *Fanuc LR Mate 200iC* robot.

Website: [Fanuc LR Mate-serie](https://www.fanuc.eu/be/nl/robots/robot-filter-pagina/lrmate-serie)

In de onderstaande figuren staat de (al geplaatste) *uFactory xArm6* robot aan de linkerkant. Daaronder staat de nieuwe robot, de *Fanuc LR Mate 200iC* (let op: dit toont alleen de robot, het is geen voorbeeld van de oplossing).

::::{grid} 2
:::{grid-item-card} 
![Image](./images/assignment3_original_robot.jpg)
:::
:::{grid-item-card}
![Image](./images/assignment3_replacement_robot.jpg)
:::
::::


Bewerk daartoe het *"assignment3.urdf.xacro"* bestand in de package urdf_basics van 2_urdf directory.

Het xacro bestand van de *Fanuc LR Mate 200iC* robot bevindt zich in de volgende plaats:
* Package: fanuc_robots
* Directory: urdf
* Bestandsnaam: lrmate200ic.urdf.xacro

Voeg je urdf-xml code toe achter de regel:
```
<!-- Update the below block for assignment 3 here -->
```

Start assignment 3
```bash
ros2 launch urdf_basics visualize_assignment3.launch.py
```


