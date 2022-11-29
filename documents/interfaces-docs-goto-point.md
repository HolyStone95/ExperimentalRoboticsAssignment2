
# Interfaces Documentation -- node *go_to_point.py*

## geometry_msgs/Twist

```
Vector3  linear
Vector3  angular
```

## geometry_msgs/Vector3

```
float64 x
float64 y
float64 z
```

## nav_msgs/Odometry

```
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

## std_msgs/Header

```
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
```

## geometry_msgs/PoseWithCovariance

```
# This represents a pose in free space with uncertainty.
Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```

## geometry_msgs/Pose

```
Point position
Quaternion orientation
```

## geometry_msgs/Point

```
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```

## geometry_msgs/Quaternion

```
float64 x
float64 y
float64 z
float64 w
```

## geometry_msgs/TwistWithCovariance Message

```
# This expresses velocity in free space with uncertainty.
Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```

## std_srvs/SetBool

see also [std_srvs package](http://docs.ros.org/en/api/std_srvs/html/index-msg.html)

```
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

