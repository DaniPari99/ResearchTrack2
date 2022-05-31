# Research Track I - final assignment

------------------------------------------

Parisi Daniele Martino 4670964

The final assignment requires to write the code for moving a robot in the environment initially unknown. Three modalities for moving the robot are required:
* the robot has to reach a goal chosen by the user
* the robot is driven by the user through the keyboard
* the robot is driven by the user with the assistence of a collision avoidence architecture

This is the final assignment's environment visualized with Gazebo: 
![MicrosoftTeams-image(4)](https://user-images.githubusercontent.com/62515616/152884108-da0032b9-8a4f-4565-8503-6839d839a8f2.png)

Initially the robot does not have a map of the environment, but it can build it through the laser scanner and thanks to the **gmapping** package. The final map built by the robot is visualized in Rviz as follows:

![MicrosoftTeams-image(3)](https://user-images.githubusercontent.com/62515616/152884728-46a1fe86-923b-4e8d-9b25-a15c8540d695.png)


## How to run the simulation
In order to launch different nodes with only a command I implemented a launch file called **final.launch** that is the following:
```
<?xml version="1.0"?>

<launch>
    <include file="$(find final_assignment)/launch/simulation_gmapping.launch" />
    <include file="$(find final_assignment)/launch/move_base.launch" />
    <node pkg="final_assignment" type="user_interface" name="user_interface" output="screen" required="true" launch-prefix="xterm -e"/>
    <node pkg="final_assignment" type="server" name="server" output="screen" required="true" launch-prefix="xterm -e"/>
     <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop" output="screen" launch-prefix="xterm -e" respawn="true">
    <remap from="cmd_vel" to="new_cmd_vel"/>
   </node>
</launch>
```
it contains 2 nested launch file: **simulation_gmapping.launch** and **move_base.launch** which respectively launch the simulation environment and provide some tools for moving the robot in the environment.

It can be launched with the command:

```bash
$ roslaunch final_assignment final.launch
```
## Project structure
![gr](https://user-images.githubusercontent.com/62515616/152952323-427302ad-438c-4cdf-b08c-16b1f12a519a.png)
As we can see from this graph, we have 9 nodes overall, even if **/user_interface** and **gazebo_gui** are not relationed with nothing.
I choose to remap **/cmd_vel** topic from /cmd_vel to **/new_cmd_vel** according to achieve the specification regarding the 3rd modality for driving the robot avoiding the obstacles. In this way the **/server** node is subscribed to the remapped topic **/new_cmd_vel** and can check if the current velocities imposed by the user can cause collisions. In this case the values of the velocities are modified and published to the actual **/cmd_vel** according to stop the robot in difficulty situations. **/server** node is linked to **/move_base** according to publish on **/move_base/goal** the goal chosen by the user in the 1st modality; for this purpose I also implemented a subscriber to the topic **/move_base/feedback** according to get the real-time position of the robot and check if the goal is reain this case I have a publisher to the topic **/move_base/** for cancelling the current goal once is reached.
**/server** is also connected to **/gazebo** for receiving the laser scanner data on **/scan** topic.

## Pseudo-code
The behaviour of the most important node of the assignment: **server** can be summarize by the following pseudo-code:

Whenever the user types a correct command in the user_interface shell the serviceCallback is executed:
```
if command = 1

    print "what target do you want to achieve?"
    
    get from input keyboard the goal chosen
    
    set the bool variable 'goal' to true
    
    set the goal_msg fields pos_x and pos_y with the chosen coordinates
    
    publish goal_msg on topic move_base/goal
 
else if command = 2

    set the bool variable manDrive to true
    
    set collisionAvoidence variable to false
    
else if command = 3

    set collisionAvoidence variable to true
    
    set the bool variable manDrive to false
```
Whenever the user try to change the velocities from the teleop_twist_keyboard 'getVelCallback' is executed:
```
if we are in the 1st modality

    the keyboard is not considered
    
else if we are in the 2nd modality

    the keyboard is considered and the velocities are published on topic /cmd_vel
    
else if we are in the 3rd modality

    the message is saved in a global variable and maybe it will be corrected by the collisionAvoidenceCallback before being published
    
```
Whenever the position of the robot changes 'currentPositionCallback' is executed:
```
if goal is set to true

    get the current position of the robot
    
    calculate the distance between the goal and the robot position
    
    if this distance is less than or equal to a threshold
    
        print "goal reached"
        
        cancel the goal

```
Whenever a message from laserScan is received and we are in the 3rd modality 'collisionAvoidenceCallback' is executed:
```
divide the array 'ranges' returned by the LaserScan topic in 5 sector (right, front_right, front, fron_left, left) and fill them appropriately

if min_front_dist is less than a threshold

    if linear velocity is greater than 0 and angular velocity is equal to zero
    
        stop the robot
        
if min_front_left_dist is less than a threshold

    if linear velocity is greater than 0 and angular velocity is greater than zero
    
        stop the robot
        
if min_front_right_dist is less than or equal to a threshold

    if linear velocity is greater than 0 and angular velocity is less than zero
    
        stop the robot
        
if min_left_dist is less than or equal to a threshold

    if linear velocity is equal to 0 and angular velocity is greater than zero
    
        stop the robot
        
if min_right_dist is less than or equal to a threshold

    if linear velocity is equal to 0 and angular velocity is less than zero
    
        stop the robot
        
publish corrected velocity on topic /cmd_vel

```
## System limitations and possible improvements
### Limitations:
* it is not considered the case in which the user chooses a goal that is not reachable, so in this case the robot will move infinitely to search the target position as long as the user either changes goal or modality
* in the case in which the user sends 2 or more targets one immediately after the other, the robot will try to reach the last one chosen

### Possible improvements
* it can be checked if the goal chosen is reachable or not and in this case printing a message in the console like 'the target chosen is not reachable, retype'
* it can be implemented a logic that stores in a queue more than one goal chosen, according to reach them sequentially

### Documentation
https://danipari99.github.io/ResearchTrack2/






