# Projekt-DL
Projekt

<br> <br>

This project contains codes for two boards:
* Main board: STM Nucleo-L053R8
* Additional board: Freescale KL05Z32


## Project Description:
This is Line Follower project, so this is a line-tracking robot (this robot tracks a black line). The line tracking was implemented by using the PID algorithm (which is used for regulation). In addition, this vehicle has an infrared remote control mode.
<br>
<br>
 <br>

### Important Info about project:

The projects were done in 2 different IDEs:
<br>
•	Project **Line_Follower_PID** in _CubeIDE_
<br>
•	Project **Diody_proj** in _KeiluVision5_
<br>
<br>


Main board is used for Line Follower code
Additional board controls a display that displays driving mode (Line Follower mode or remote control mode)
    and lights up with an LED:
<br>                         - Red color - driving right
<br>                         - Green color - driving left
<br>                         - Yellow color - driving straight
<br>                         - Blue color - IR remote control mode

Additional board "knows" what mode is using by STM board because there is communication between the boards.


