# Robot Maze Navigation
This is a navigation system coded in assembly language that allows the HCS12 microcontroller-based _eebot_ to autonomously navigate a maze. The primary objectives included enabling the robot to make decisions at intersections and effectively handle dead ends, which led to successful navigation of the maze. 

Software subroutines and a finite state machine were developed in assembly programming to coordinate these components seamlessly for smooth and efficient functionality. Six pairs of LEDs and cadmium sulphide sensors attached to the bottom of the bot detected the black line path. Each intersection led to either a dead end or the next part of the maze. The _eebot’s_ LCD screen was used to display critical information for testing and debugging.

![IMG_3800](https://github.com/user-attachments/assets/a40f7084-9b52-42c2-bea7-cbb247937bd3)

The maze, shown below, consisted of a black taped line forming several turns and intersections.

![image](https://github.com/user-attachments/assets/11f33103-dee7-461e-84fd-219564e5be7d)

## Algorithm
The _eebot_ begins its navigation with the actuation of the front bumper, initiating movement in the forward state following a finite state machine (FSM) architecture. As it moves, its sensors detect the maze path and continuously send readings to the program, enabling real-time adjustments based on a negative feedback system. Each sensor has a threshold and tolerance range, allowing the bot to remain aligned with the maze path. When approaching turns, the bot detects deviations from the black line and adjusts its bearing accordingly. At intersections, it evaluates sensor readings to determine its position and selects the leftmost path by default. If it encounters a dead end, the bumper actuation triggers a reversal and re-navigation to the previous intersection, allowing the bot to take the correct path.
