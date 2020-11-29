#!/bin/sh
rostopic pub -l --once /house_light aws_robomaker_small_house_world/house_light_msgs "{light_number:[1,2,3,4,5,6,7],range: 20.0}"
