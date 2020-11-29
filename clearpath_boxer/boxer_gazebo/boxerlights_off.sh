#!/bin/sh
rostopic pub -l --once /boxer_light_toggle std_msgs/Float32 "data: 0.0"
