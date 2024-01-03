# Fuzzy-Systems-Robot-Controller

There are 4 python files found in this repository, all with the purpose of controlling the behavior of a small robot with 2 wheels and a LiDAR Scanner, each code instructs the robot to follows its right edge wall, avoids obstacles or combine both behaviors. There are also videos showcasing the different codes in action, except for "fc_combo" as the video was too heavy. 

- Pid.py, is a a simple Proportional–integral–derivative controller (PID) tuned so the robot followed the wall located on its right edge from various starting positions. The controller can be found in the function “PIDcontroller”

- fc_ref.py, implements a complete fuzzy system coded manually. Meaning no external libraries outside were used for the controllers development. It goes through the different stages of fuzzy systems from fuzzyfing the input, declaring membership functions, establishing a rule base, etc. This code used two readings from the right part of the LiDAR Scanner to determine the angular and motor speed needed to follow a right wall and not crash into it. It can even turn corners.

- fc_oa.py, uses the same fuzzy system as fc_ref.py just modifying the membership functions

- fc_combo.py, combines both behaviors using context blending or subsumption, to create a robot that follows a right edge wall and can avoid obstacles. 
