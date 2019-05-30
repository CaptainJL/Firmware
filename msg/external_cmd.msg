# Command from external Source - eg NVIDIA Tegra TX1

# Validity?
bool cmd_valid

# Command Type
uint8 cmd	   # 0 = no command, 1 = hover, 2 = full velocity control

# Control Commands
float32 x
float32 y
float32 z
float32 vx
float32 vy
float32 vz
float32 ax
float32 ay
float32 az

float32 yaw_sp

# TX2 Reference Velocities
float32 vbx_tx2
float32 vby_tx2
float32 vbz_tx2
float32 vix_tx2
float32 viy_tx2
float32 viz_tx2
