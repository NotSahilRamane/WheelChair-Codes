# A standard object message for perception

geometry_msgs/Point position # position in x, y ,z in meters
std_msgs/String class # string for the actual object class, i.e. tree, pedestrian, car, truck, etc.

std_msgs/String status # wether the object is static/dynamic in nature etc.
geometry_msgs/Pose2D object_state_dt # Velocity in x, y, theta of a dynamic object

std_msgs/Int16 id # unique object id for every object