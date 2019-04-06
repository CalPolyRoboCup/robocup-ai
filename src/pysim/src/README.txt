This is a pygame based simulator for the robocup competition. 

use the Pysim.add_action(action, index, is_blue) function of the Pysim object to place actions on robots.
Directly adding an action onto a robot risks peeking at the unnoised true parameters. 
Currently noise of magnitude 0 is added.

use step() function to advance the simulation. Step has an optional argument key_points that
can be used for debuging. pass a list of points or a list of (point, size) to have them plotted 
in the draw() step. Negative values of size will be plotted with a different color.

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