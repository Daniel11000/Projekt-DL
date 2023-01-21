# Projekt-DL
Projekt

This project contains codes for two boards:
Main board: STM Nucleo-L053R8
Additional board: Freescale KL05Z32

Main board is used for Line Follower code
Additional board controls a display that displays driving mode (Line Follower mode or remote control mode)
    and lights up with an LED:
<br>                         - Red color - driving right
<br>                         - Green color - driving left
<br>                         - Yellow color - driving straight
<br>                         - Blue color - IR remote control mode

Additional board "knows" what mode is using by STM board because there is communication between the boards.


