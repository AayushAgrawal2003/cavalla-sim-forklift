# Drive Commands
float32 drive_speed       # Normalized -1.0 to 1.0
float32 steering_angle    # Normalized -1.0 to 1.0 (maps to +/- pi/2 rad on hardware)

# Fork Commands (Auxiliary Axes)
float32 lift_speed        # Normalized -1.0 (down) to 1.0 (up)
float32 tilt_speed        # Normalized -1.0 (forward) to 1.0 (back)
float32 side_shift_speed  # Normalized -1.0 (left) to 1.0 (right)
float32 fork_spread_speed # Normalized -1.0 (in) to 1.0 (out)

# Safety
bool estop_active         # True triggers immediate brake on CAN bus

# Drive Curve Parameters (0.0 = use driver defaults)
float32 accel_time_s
float32 decel_time_s
bool brake_interlock_released    # True releases the electromagnetic brake (toggled by menu button)
