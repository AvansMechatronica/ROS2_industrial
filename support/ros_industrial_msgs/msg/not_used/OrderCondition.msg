uint8 KITTING=0
uint8 ASSEMBLY=1
uint8 COMBINED=2

string id
uint8 type
bool priority
ros_industrial_msgs/KittingTask kitting_task 
ros_industrial_msgs/AssemblyTask assembly_task
ros_industrial_msgs/CombinedTask combined_task
ros_industrial_msgs/Condition condition