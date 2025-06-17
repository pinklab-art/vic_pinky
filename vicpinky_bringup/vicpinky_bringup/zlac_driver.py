import serial
import struct
import threading
import time

class ZLACDriver:
    def __init__(self, port, baudrate, modbus_id):
        self.port = port
        self.baudrate = baudrate
        self.modbus_id = modbus_id
        self.ser = None
        self.lock = threading.Lock()

        self.READ = 0x03
        self.WRITE = 0x06
        self.MULTI_WRITE = 0x10
        self.CONTROL_WORD = 0x200E
        self.MOTOR_ENABLE = 0x0008
        self.MOTOR_DISABLE = 0x0007
        self.CONTROL_MODE = 0x200D
        self.VEL_MODE = 0x0003
        self.SET_L_RPM = 0x2088
        self.GET_RPM = 0x20AB
        self.GET_ENCODER_PULSE = 0x20A6

    def _calculate_crc(self, data: bytearray) -> bytes:
        """Calculates CRC-16 for Modbus."""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def _send_command(self, command_bytes: bytes, read_bytes: int = 0):
        """Sends a command and returns the full response if successful."""
        with self.lock:
            try:
                self.ser.flushInput()
                self.ser.write(command_bytes)
                if read_bytes > 0:
                    response = self.ser.read(read_bytes)
                    if len(response) == read_bytes and self._calculate_crc(response[:-2]) == response[-2:]:
                        return response
                    # print(f"CRC/Length error. Rcvd: {response.hex() if response else 'None'}")
                    return None
                return True
            except (serial.SerialException, OSError) as e:
                print(f"Serial communication error: {e}")
                return None

    def begin(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            return True
        except serial.SerialException:
            return False

    def enable(self):
        cmd_body = struct.pack('>BBHH', self.modbus_id, self.WRITE, self.CONTROL_WORD, self.MOTOR_ENABLE)
        return self._send_command(cmd_body + self._calculate_crc(cmd_body), 8) is not None

    def disable(self):
        cmd_body = struct.pack('>BBHH', self.modbus_id, self.WRITE, self.CONTROL_WORD, self.MOTOR_DISABLE)
        return self._send_command(cmd_body + self._calculate_crc(cmd_body), 8) is not None

    def set_vel_mode(self):
        cmd_body = struct.pack('>BBHH', self.modbus_id, self.WRITE, self.CONTROL_MODE, self.VEL_MODE)
        return self._send_command(cmd_body + self._calculate_crc(cmd_body), 8) is not None

    def set_double_rpm(self, l_rpm: int, r_rpm: int) -> bool:
        """
        [FIXED] This function now correctly builds the Modbus RTU frame
        for function 0x10 (Write Multiple Registers) exactly like the C++ source.
        """
        r_rpm_neg = -r_rpm
        
        # Frame format: [ID, Func, StartAddr, NumRegs, ByteCount, Data..., CRC]
        cmd_body = bytearray()
        cmd_body.append(self.modbus_id)
        cmd_body.append(self.MULTI_WRITE)
        cmd_body.extend(struct.pack('>H', self.SET_L_RPM)) # Start Address 0x2088
        cmd_body.extend(struct.pack('>H', 2))              # Quantity of Registers
        cmd_body.append(4)                                 # Byte Count
        cmd_body.extend(struct.pack('>h', int(l_rpm)))     # L_RPM data
        cmd_body.extend(struct.pack('>h', int(r_rpm_neg))) # R_RPM data
        
        response = self._send_command(cmd_body + self._calculate_crc(cmd_body), 8)
        return response is not None

    def get_rpm(self):
        """[FIXED] Correctly requests 4 registers and expects a 13-byte response."""
        cmd_body = struct.pack('>BBHH', self.modbus_id, self.READ, self.GET_RPM, 4)
        response = self._send_command(cmd_body + self._calculate_crc(cmd_body), 13)
        if response:
            l_rpm = struct.unpack('>h', response[3:5])[0] / 10.0
            r_rpm = -struct.unpack('>h', response[5:7])[0] / 10.0
            return l_rpm, r_rpm
        return None, None

    def get_position(self):
        """[FIXED] Correctly requests 5 registers and unpacks from the correct indices."""
        cmd_body = struct.pack('>BBHH', self.modbus_id, self.READ, self.GET_ENCODER_PULSE, 5)
        response = self._send_command(cmd_body + self._calculate_crc(cmd_body), 15)
        if response:
            encoder_l = struct.unpack('>i', response[5:9])[0]
            encoder_r = -struct.unpack('>i', response[9:13])[0]
            return encoder_l, encoder_r
        return None, None
        
    def terminate(self):
        """Stops and disables the motor, then closes the port."""
        print("Terminating motor...")
        if self.ser and self.ser.is_open:
            self.set_double_rpm(0, 0)
            time.sleep(0.05)
            self.disable()
            self.ser.close()
            print("Motor terminated.")

