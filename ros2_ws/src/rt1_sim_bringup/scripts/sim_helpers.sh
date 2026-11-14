#!/usr/bin/env bash
# ==============================================================================
#  FILE: rt1_sim_bringup/scripts/sim_helpers.sh
# ==============================================================================
#  Target:  Shell shortcuts for manual G2 / Phase 1 testing. Source, don't run:
#           source /workspace/ros2_ws/src/rt1_sim_bringup/scripts/sim_helpers.sh
#  Inputs & Outputs:
#    arm q1 q2 q3 q4   joint-space move (rad), 3 s, via /arm_controller action
#    grip F            grip force (N): F > 0 squeeze, F < 0 open (gripper_force_node)
#    fingers           /joint_states positions: [left, right, j1, j2, j3, j4]
#    plates            block plate joint positions (m), 0 = rest, + = compressed
#    spawn k c [m]     spawn mystery_block at the marker: stiffness k (N/m),
#                      damping c (N*s/m), mass m (kg, default 0.05)
#    unspawn           remove mystery_block
#  Poses (gripper pitched 0.1 rad down, pad centre at x = 0.20 m):
#    home      0 -1.0   0.3   0.7
#    pregrasp  0 -0.388 1.139 -0.652   (pad z 0.09)
#    grasp     0  0.352 1.237 -1.489   (pad z 0.035)
#    lift      0  0.119 1.246 -1.264   (pad z 0.05)
# ==============================================================================

BLOCK=/workspace/ros2_ws/install/rt1_sim_bringup/share/rt1_sim_bringup/urdf/mystery_block.sdf.xacro

arm(){ ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4], points: [{positions: [$1, $2, $3, $4], time_from_start: {sec: 3}}]}}" \
  > /dev/null && echo "arm: reached"; }

grip(){ ros2 topic pub --once /gripper/force_cmd std_msgs/msg/Float64 "{data: $1}" > /dev/null && echo "grip: F=$1 N"; }

fingers(){ ros2 topic echo /joint_states --once --field position 2>/dev/null | grep array; }

plates(){ gz topic -e -n 1 -t /world/sorting_arena/model/mystery_block/joint_state | grep -E 'name: "plate|position'; }

spawn(){ ros2 run ros_gz_sim create -world sorting_arena -name mystery_block \
  -string "$(xacro $BLOCK mass:=${3:-0.10} k:=$1 c:=$2 mu:=0.8 ${4:-})" -x 0.20 -y 0.0 -z 0.026; }
  
unspawn(){ gz service -s /world/sorting_arena/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'name: "mystery_block" type: MODEL'; }

home(){ arm 0 -1.0 0.3 0.7; }
pregrasp(){ arm 0 -0.388 1.139 -0.652; }
grasp(){ arm 0 0.352 1.237 -1.489; }
lift(){ arm 0 0.119 1.246 -1.264; }

echo "sim_helpers loaded: arm grip fingers plates spawn unspawn home pregrasp grasp lift"