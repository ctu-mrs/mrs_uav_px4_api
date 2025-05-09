#/bin/bash

G1=${1:-0.0}
G2=${2:-0.0}
G3=${3:-3.0}
G4=${4:-0.0}

echo "going to $G1 $G2 $G3 $G4"

ros2 service call /uav1/control_manager/goto mrs_msgs/srv/Vec4 '{"goal": ['$G1', '$G2', '$G3', '$G4']}'
