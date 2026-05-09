# TPDO1 - Drive (CAN 0x183)
int16   motor_speed_rpm       # signed, -4000 to 4000; positive = forward
float32 motor_current_amps    # 0.1A resolution, 0-1000A range
uint8   drive_fault_code      # refer to Curtis F2A Fault Manual; 0 = no fault
uint8   battery_percent       # lead acid battery level, 0-100

# TPDO2 - Odometer (CAN 0x283)
uint32  odometer_cm           # cumulative distance since controller power-on; always increases

# TPDO2 - Switch status bits (drive_switch_status byte)
bool estop_active             # true = e-stop is pressed (bit 0 inverted, active low in protocol)
bool interlock_on             # true = interlock is engaged (bit 1)
bool forward_switch           # true = forward drive direction commanded (bit 2)
bool reverse_switch           # true = reverse drive direction commanded (bit 3)
bool mode_auto                # true = autonomous mode, false = manual/teleop (bit 6)

# TPDO2 - Output status bits (drive_output_status byte)
bool electromagnetic_brake    # true = brake is clamped/engaged, false = released (bit 1)
