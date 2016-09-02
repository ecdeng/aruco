#!/bin/bash

# chmod 755 file.sh
rostopic pub -1 /arucoDelay_topic std_msgs/Float32 '0.17'
