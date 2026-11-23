# Expert FSM status, published at 50 Hz on /fsm/status.
uint8 IDLE=0
uint8 HOME=1
uint8 APPROACH=2
uint8 PINCH_TEST=3
uint8 MICRO_LIFT=4
uint8 ROUTE_BIN=5
uint8 RELEASE=6
uint8 RETURN=7
uint8 DONE=8
uint8 FAILED=9

std_msgs/Header header
uint32 episode_id
uint8 state
string state_name
string step_label             # current sub-step inside the state
string failure_code           # empty unless FAILED

float64 grip_force_cmd_n      # last commanded squeeze (negative = open)

bool decision_made
string decided_stiffness      # "soft" | "stiff" (from eps_vis only)
string decided_mass           # "light" | "heavy" (from slip only)
string decided_bin            # "blue_bin" | "red_bin"
float64 eps_vis_at_decision
bool slip_detected
uint8 regrasp_count
bool decision_matches_truth   # decided_bin == ground-truth target_bin