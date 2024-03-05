The code above is the code used on our robot before the competition.

The code is a "Multi State" Control system, which means the robot behaves differently 
depending on how far away from the ball it is, if it is on a line, if the ball is in sight etc.

For reading the line sensor, we are using a custom function that reads 2 analog values
from the 2, 16 channel multiplexors (32 sensors in total). Also it uses a fully automatic
calibration system that we developed from scratch.

The code is fully configurable so its easy to adjust parameters such as motor speed,
if the robot should use the line sensors, max pid gain and a few more things.

For direction control we use a special function that automaticaly calculates
motor speed based on the desiered direction the robot should go in, the desiered speed 
while also taking into account the compas direction.

The compass is a BNO055 9-axis Gyro mounted on the same board our ESP is on.

All our robot parts (apart from the line senzor) are controled via can bus, which means
that we don't have to worry about messy communication wires.


