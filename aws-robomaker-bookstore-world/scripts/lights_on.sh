#!/bin/sh
rostopic pub -l --once /bookstore_lights aws_robomaker_bookstore_world/bookstore_light_msgs "{light_number:[1,2,3,4,5,6,7,8,9],range: 20.0}"
