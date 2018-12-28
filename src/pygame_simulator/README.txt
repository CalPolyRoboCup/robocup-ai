This is a pygame based simulator for the robocup competition. 

similar to GRsim networking code with the same API should be a plug and play replacement.
This interface should be common to all "simulators" so that code can be easily tested in GSrim,
Pysim, and physical robots without issue.

Example code at the bottom of script. Makes all of the blue robots controlable via keyboard controls
wasd to move
qe to rotate
SPACE to shoot


TODO:
chipping not implamented
physics not reprasentative of phisical robot
Noise added in step() return, but the raw 