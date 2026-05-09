import can
import struct
import time

def force_lower():
    print("--- FORKLIFT LOWERING DIAGNOSTIC ---")
    print("Trying to connect to can0...")
    
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=250000)
        print("Connected.")
    except Exception as e:
        print(f"ERROR: Could not connect to CAN bus: {e}")
        return

    # RPDO1 ID (0x200 + 0x03)
    rpdo1 = 0x203
    rpdo2 = 0x303

    print("\nWARNING: This will command the lowering valve to OPEN at 100% PWM.")
    print("Ensure the area under the forks is clear.")
    print("Press CTRL+C immediately to stop.")
    time.sleep(2)

    try:
        while True:
            # --- CONSTRUCTING THE MESSAGE BASED ON C++ SNIPPET ---
            # void ForkLogic::lower_valve(int valve_0_200)
            # {
            #   set_pump_pwm(0);                 -> RPDO2 Pump = 0
            #   drive_.set_lift_solenoid(false); -> RPDO1 Bit 5 = 0
            #   drive_.set_lower_solenoid(true); -> RPDO1 Bit 4 = 1
            #   drive_.set_descent_valve(valve); -> RPDO1 Byte 7 = 200
            # }

            # Flags: 
            # Bit 0 (Interlock) = 1 (Assume we need this to enable the controller?)
            # Bit 4 (Lower)     = 1
            # Bit 5 (Lift)      = 0
            flags = (1 << 0) | (1 << 4)  # 0x11 (17 decimal)

            rpm = 0
            accel = 10
            decel = 10
            steer = 0
            
            # FORCE MAX PWM (200) to rule out "weak signal" issues
            lowering_pwm = 50

            # Pack RPDO1
            data1 = struct.pack('< B H B B h B', flags, rpm, accel, decel, steer, lowering_pwm)
            bus.send(can.Message(arbitration_id=rpdo1, data=data1, is_extended_id=False))

            # Pack RPDO2 (Pump RPM = 0)
            data2 = struct.pack('< B B B H B B B', 0, 0, 0, 0, 5, 5, 0)
            bus.send(can.Message(arbitration_id=rpdo2, data=data2, is_extended_id=False))

            print(f"Sent: Flags=0x{flags:02X} | PWM={lowering_pwm} | Pump=0", end='\r')
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nSTOPPING...")
        # Send Stop Command
        stop_data = struct.pack('< B H B B h B', 0, 0, 0, 0, 0, 0)
        bus.send(can.Message(arbitration_id=rpdo1, data=stop_data, is_extended_id=False))
        print("Forks Stopped.")

if __name__ == "__main__":
    force_lower()