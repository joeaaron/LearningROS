#!/bin/bash
rostopic pub -1 /target_offset_lighthouse geometry_msgs/Pose2D -- '{x: -0.3,y: 0.0,theta: 0.0}'
