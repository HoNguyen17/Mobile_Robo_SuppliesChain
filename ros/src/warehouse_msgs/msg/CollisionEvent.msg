# Published by Unity on /sim/collision when the robot touches something.
# Unity debounces repeated contact with the same object to 1 s.

string TAG_WALL=Wall
string TAG_SHELF=Shelf
string TAG_OBSTACLE=Obstacle
string TAG_NPC=NPC

std_msgs/Header header             # stamp = sim time of first contact
string other_tag                   # one of the TAG_* constants
geometry_msgs/Point contact        # contact point, map frame
