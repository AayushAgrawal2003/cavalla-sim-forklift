import can
import struct
import math

class MBV15Interface:
    def __init__(self, channel='can0', bitrate=250000, is_mock=False):
        self.connected = False
        self.bus = None
        
        if not is_mock:
            try:
                self.bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=bitrate)
                self.connected = True
            except OSError as e:
                print(f"[MBV15] CAN Bus error: {e}. Running in disconnected mode.")

        self.node_id = 0x03
        self.rpdo1_id = 0x200 + self.node_id
        self.rpdo2_id = 0x300 + self.node_id
        self.tpdo1_id = 0x180 + self.node_id
        self.tpdo2_id = 0x280 + self.node_id
        self.tpdo3_id = 0x380 + self.node_id

        self.state = {
            # TPDO1
            'motor_rpm': 0,
            'current_amps': 0.0,
            'drive_fault': 0,
            'battery_percent': 0,
            # TPDO2
            'odometer_cm': 0,
            'estop_active': True,
            'interlock_on': False,
            'forward_switch': False,
            'reverse_switch': False,
            'mode_auto': False,
            'electromagnetic_brake': True,
            # TPDO3
            'steering_rad': 0.0,
            'steer_fault': 0,
        }

    def send_commands(self, drive_norm, steer_norm, lift_norm, tilt_norm, shift_norm,
                      accel_s=1.0, decel_s=1.0, interlock_released=False):
        if not self.connected: return

        # ==========================================
        # RPDO1: FLAGS & MOTION
        # ==========================================
        flags = 0x01 if interlock_released else 0x00  # Bit 0: Brake interlock release
        
        if drive_norm > 0.02:   flags |= (1 << 1)
        elif drive_norm < -0.02: flags |= (1 << 2)

        # --- HYDRAULIC FLAGS FIX ---
        is_lowering = lift_norm < -0.05
        is_lifting  = lift_norm > 0.05
        is_aux      = abs(tilt_norm) > 0.5 or abs(shift_norm) > 0.5

        if is_lowering:
            flags |= (1 << 4)  # Bit 4: Lowering Valve Enable
            
        elif is_lifting:
            flags |= (1 << 5)  # Bit 5: Lift Solenoid Enable (STRICTLY for lifting)

        # Drive RPM
        rpm = int(abs(drive_norm) * 4000)
        rpm = max(0, min(rpm, 4000))

        # Accel/Decel
        accel = max(0, min(255, int(accel_s * 10)))
        decel = max(0, min(255, int(decel_s * 10)))

        # Steering
        MAX_STEER = 9000
        steer_can = int(steer_norm * MAX_STEER)
        steer_can = max(-MAX_STEER, min(MAX_STEER, steer_can))

        # Lowering PWM (Scaled 40-200)
        lowering_pwm = 0
        if is_lowering:
            raw_val = abs(lift_norm)
            min_pwm = 40
            max_pwm = 200
            pwm_val = min_pwm + (raw_val * (max_pwm - min_pwm))
            lowering_pwm = int(pwm_val)
            lowering_pwm = max(0, min(lowering_pwm, 200))

        data1 = struct.pack('< B H B B h B', flags, rpm, accel, decel, steer_can, lowering_pwm)
        self.bus.send(can.Message(arbitration_id=self.rpdo1_id, data=data1, is_extended_id=False))

        # ==========================================
        # RPDO2: PUMP RPM & AUX VALVES
        # ==========================================
        valve_flags = 0x00
        if tilt_norm > 0.5:   valve_flags |= (1 << 0)
        elif tilt_norm < -0.5: valve_flags |= (1 << 1)
        if shift_norm > 0.5:   valve_flags |= (1 << 2)
        elif shift_norm < -0.5: valve_flags |= (1 << 3)

        pump_rpm = 0
        
        if is_lifting:
            pump_rpm = 1000 + int(lift_norm * 2500)
        elif is_aux:
            pump_rpm = 2000  # Spin the pump for aux, but without opening the lift solenoid!
            
        if is_lowering:
            pump_rpm = 0

        pump_rpm = max(0, min(5000, pump_rpm))

        data2 = struct.pack('< B B B H B B B', valve_flags, 0, 0, pump_rpm, 5, 5, 0)
        self.bus.send(can.Message(arbitration_id=self.rpdo2_id, data=data2, is_extended_id=False))

    def stop_all(self):
        if not self.connected: return
        self.bus.send(can.Message(arbitration_id=self.rpdo1_id, data=struct.pack('< B H B B h B', 0, 0, 0, 0, 0, 0)))
        self.bus.send(can.Message(arbitration_id=self.rpdo2_id, data=struct.pack('< B B B H B B B', 0, 0, 0, 0, 5, 5, 0)))

    def read_feedback(self, timeout=0.0):
        if not self.connected: return self.state
        while True:
            msg = self.bus.recv(timeout=timeout)
            if msg is None: break
            if msg.arbitration_id == self.tpdo1_id: self._parse_tpdo1(msg.data)
            elif msg.arbitration_id == self.tpdo2_id: self._parse_tpdo2(msg.data)
            elif msg.arbitration_id == self.tpdo3_id: self._parse_tpdo3(msg.data)
        return self.state

    def _parse_tpdo1(self, data):
        if len(data) < 8: return
        rpm, current_raw, fault, batt, temp = struct.unpack('< h H B B h', data)
        self.state['motor_rpm'] = rpm
        self.state['current_amps'] = current_raw * 0.1
        self.state['drive_fault'] = fault
        self.state['battery_percent'] = batt

    def _parse_tpdo2(self, data):
        if len(data) < 8: return
        odom, pump_io, ctrl_io, drive_sw, drive_out = struct.unpack('< I B B B B', data)
        self.state['odometer_cm'] = odom
        # drive_switch_status bits (active-low e-stop)
        self.state['estop_active']    = (drive_sw & (1 << 0)) == 0
        self.state['interlock_on']    = bool(drive_sw & (1 << 1))
        self.state['forward_switch']  = bool(drive_sw & (1 << 2))
        self.state['reverse_switch']  = bool(drive_sw & (1 << 3))
        self.state['mode_auto']       = bool(drive_sw & (1 << 6))
        # drive_output_status bits
        self.state['electromagnetic_brake'] = bool(drive_out & (1 << 1))

    def _parse_tpdo3(self, data):
        if len(data) < 2: return
        angle_raw, = struct.unpack_from('< h', data, 0)
        self.state['steering_rad'] = math.radians(angle_raw / 100.0)
        if len(data) >= 3:
            self.state['steer_fault'] = data[2]