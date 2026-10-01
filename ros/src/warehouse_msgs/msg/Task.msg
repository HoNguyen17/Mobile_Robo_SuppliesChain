# One fulfilment task: move one item from its shelf slot to a drop-off pose.
# Poses are in the map frame (metres, radians).

string CATEGORY_STANDARD=Standard
string CATEGORY_HEAVY=Heavy
string CATEGORY_FRAGILE=Fragile

int32 task_id
int32 item_id
string category                    # one of the CATEGORY_* constants
geometry_msgs/Pose2D shelf_pose
geometry_msgs/Pose2D dropoff_pose
