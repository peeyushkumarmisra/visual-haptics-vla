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

echo "sim_helpers loaded: arm grip fingers plates spawn unspawn home pregrasp grasp lift sq lift_test"
# == G2 verification tools (14 Nov) ==
# sq k F [extra xacro args]: squeeze a block of stiffness k (N/m) at F (N),
#   damping c = sqrt(k * m_plate) (zeta ~ 0.5); prints plate compression and
#   the ratio k*x/F (expected 0.995 after the explicit-spring fix).
sq(){ local k=$1 F=$2 c x
  c=$(awk -v k=$k 'BEGIN{printf "%.3f", sqrt(k*0.02)}')
  grip -1.0 >/dev/null; pregrasp >/dev/null; unspawn >/dev/null 2>&1; sleep 2
  spawn $k $c 0.10 "${3:-}" >/dev/null 2>&1; sleep 3; grasp >/dev/null
  grip 0.3 >/dev/null; sleep 2            # gentle close first: low finger damping = fast fingers
  grip $F >/dev/null; sleep 4
  x=$(plates | grep 'position: ' | awk '{print $2}' | head -1)
  awk -v k=$k -v F=$F -v x=${x:-0} 'BEGIN{printf "k=%-5s F=%-4s x=%.4f mm  pred=%.4f mm  ratio=%.4f\n", k, F, x*1000, F/k*0.995*1000, k*x/F}'; }

# lift_test m: grip a k = 400 block of mass m (kg) at 2 N, micro-lift 15 mm,
#   print how far the block rose (15 mm = held, ~0 = slipped out).
lift_test(){ local z0 z1
  grip -1.0 >/dev/null; pregrasp >/dev/null; unspawn >/dev/null 2>&1; sleep 2
  spawn 400 2.83 $1 >/dev/null 2>&1; sleep 3; grasp >/dev/null
  grip 0.3 >/dev/null; sleep 2; grip 2.0 >/dev/null; sleep 3
  z0=$(gz model -m mystery_block -p 2>/dev/null | grep -A1 XYZ | tail -1 | awk '{print $3}' | tr -d ']')
  lift >/dev/null; sleep 3
  z1=$(gz model -m mystery_block -p 2>/dev/null | grep -A1 XYZ | tail -1 | awk '{print $3}' | tr -d ']')
  awk -v m=$1 -v a=$z0 -v b=$z1 'BEGIN{printf "m=%-5s rise=%.1f mm (gripper rose 15 mm)\n", m, (b-a)*1000}'; }
