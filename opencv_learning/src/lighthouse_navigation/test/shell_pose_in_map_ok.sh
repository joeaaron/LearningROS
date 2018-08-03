#!/bin/bash
rostopic pub -1 /lighthouse_pose_in_map geometry_msgs/Pose -- '{position: {x: 0.5055,y: 0.0384,z: 0.0}, orientation: {x: 0.0,y: 0.0,z: 0.0101,w: 0.9999}}'
