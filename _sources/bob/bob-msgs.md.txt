# ROS Package [bob_msgs](https://github.com/bob-ros2/bob_msgs)
Message and Service definitions for Bob's ROS Nodes.

## Messages
### TTImage
Text to image message with payload.\
Used by bob_transformers.

**Message specification:**  [TTImage.msg](https://github.com/bob-ros2/bob_msgs/blob/main/msg/TTImage.msg)
> sensor_msgs/Image image\
  string caption\
  string json

## Services
### SetSequence
Used by package bob_face.

**Message specification:** [SetSequence.srv](https://github.com/bob-ros2/bob_msgs/blob/main/srv/SetSequence.srv)

> uint8 TYPE_CIRCULAR=0\
  uint8 TYPE_FLIPFLOP=1\
  uint8 type\
  uint8 rate\
  uint32 start\
  uint32 end\
  ---\
  string error
