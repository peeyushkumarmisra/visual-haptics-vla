# Ground truth for one episode. Published latched on /episode/info.
std_msgs/Header header

uint32 episode_id
string split                 # "train" or "eval"
uint64 seed                  # RNG seed of the whole schedule

string prompt_id             # e.g. "P_STIFF_0"
string prompt_text

string stiffness_class       # "soft" | "stiff"
string mass_class            # "light" | "heavy"
float64 stiffness_n_per_m    # plate spring k
float64 damping_n_s_per_m    # plate damper c = sqrt(k * 0.02)
float64 mass_kg

string target_bin            # "blue_bin" | "red_bin"