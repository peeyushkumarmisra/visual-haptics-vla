#!/usr/bin/env bash
# ==============================================================================
#  FILE: rt1_sim_bringup/scripts/sweep_finger_damping.sh
# ==============================================================================
#  Target:
#    G2 root-cause study. For each finger joint damping b, restart the sim
#    headless, squeeze a k = 400 N/m block at F = 2 N, and log the plate
#    compression. Shows how the static contact force depends on b, which
#    real physics says it must not.
#
#  Physics / Math Implemented:
#    Hooke prediction: x = F / k = 2 / 400 = 5.0 mm per plate.
#    Transmitted-force ratio reported per run: f = k * x / F  (1.0 = exact).
#
#  Inputs & Outputs:
#    Args : finger damping values in N*s/m, e.g.  0.05 0.2 0.5 1 2 5
#    Out  : /workspace/data/g2_finger_damping_sweep.csv
#           columns: b_ns_per_m, x_pos_m, x_neg_m, force_ratio
#    Note : edits the arm-xacro copy; the LAST value given stays in place.
#           Stop any running sim before starting.
# ==============================================================================
WS=/workspace/ros2_ws
ARM=$WS/src/rt1_sim_bringup/urdf/open_manipulator_x_arm_eqdamp.urdf.xacro
OUT=/workspace/data/g2_finger_damping_sweep.csv
K_N_PER_M=400; C_NS_PER_M=2.83; F_N=2.0

source $WS/install/setup.bash
source $WS/src/rt1_sim_bringup/scripts/sim_helpers.sh > /dev/null
# Strict mode only AFTER sourcing: ROS setup scripts reference unset variables.
set -u

# --- 1. HELPERS ---
set_finger_damping(){
  for j in gripper_left_joint gripper_right_joint; do
    sed -i "/${j}\" type=\"prismatic\"/,/<\/joint>/ s/damping=\"[0-9.]*\"/damping=\"$1\"/" $ARM
  done
}

stop_sim(){
  # SIGTERM, not SIGINT: background jobs in a script ignore SIGINT.
  kill -TERM $1 2>/dev/null
  for _ in $(seq 1 15); do kill -0 $1 2>/dev/null || break; sleep 1; done
  kill -KILL $1 2>/dev/null
  # Make sure nothing survives: stale publishers corrupt the next run.
  pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null; pkill -f gripper_force_node 2>/dev/null
  pkill -f spawner 2>/dev/null; sleep 3
}

wait_for_controllers(){
  # Up to ~90 s for all three controllers; real-time factor is ~0.5.
  for _ in $(seq 1 45); do
    n=$(ros2 control list_controllers 2>/dev/null | grep -c " active")
    [ "$n" -ge 3 ] && return 0
    sleep 2
  done
  return 1
}

# --- 2. SWEEP ---
[ -f $OUT ] || echo "b_ns_per_m,x_pos_m,x_neg_m,force_ratio" > $OUT

for b in "$@"; do
  echo "=== finger damping b = $b ==="
  set_finger_damping $b
  ros2 launch rt1_sim_bringup sim_arena.launch.py gui:=false > /tmp/sim_b${b}.log 2>&1 &
  SIM=$!

  if ! wait_for_controllers; then
    echo "b=$b: controllers did not come up, see /tmp/sim_b${b}.log"; stop_sim $SIM; continue
  fi
  sleep 3

  # Same squeeze as every G2 run: open, approach, spawn, grasp, squeeze, settle.
  home > /dev/null; grip -1.0 > /dev/null; pregrasp > /dev/null
  spawn $K_N_PER_M $C_NS_PER_M > /dev/null 2>&1; sleep 3
  grasp > /dev/null; grip $F_N > /dev/null; sleep 4

  read xp xn <<< "$(plates | grep 'position: ' | awk '{print $2}' | head -2 | tr '\n' ' ')"
  ratio=$(awk -v k=$K_N_PER_M -v x="$xp" -v f=$F_N 'BEGIN{printf "%.4f", k*x/f}')
  echo "$b,$xp,$xn,$ratio" | tee -a $OUT

  stop_sim $SIM
done
echo "done -> $OUT"