#!/bin/bash
# chmod 755 file.sh
#  IsLoadFromFile

# <0 : on line
# >0 : load file
# 1 2 3 / -1 -2 -3 : method

# recommend : '2' / ' -2'
rostopic pub -1 /calibInstall_topic std_msgs/Float32 ' -2'
