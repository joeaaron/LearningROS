#!/bin/bash
rostopic pub -1 /lighthouse_pose_in_map geometry_msgs/Pose -- '{position: {x: 0.0,y: 0.0,z: 0.0}, orientation: {x: 0.0,y: 0.0,z: 0.0,w: 0.0}}'
