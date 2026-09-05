from cobs import cobs
from anycrc import Model
from serial import Serial
from enum import IntEnum
import logging
import struct
import ctypes

CRC = Model("CRC8-SMBUS")   # poly=0x07, init=0x00, ref_in=False, ref_out=False, xor_out=0x00


class PacketType(IntEnum):
    STATUS = 1
    IMU = 2
    TOF = 3
    CONTROL = 4
    PID = 5


class MotorState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("angle", ctypes.c_float),
        ("effort", ctypes.c_float),
    ]

    def __repr__(self):
        fields = {name: getattr(self, name) for name, _ in self._fields_}
        return f"{self.__class__.__name__}({fields})"


class PidState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("limit", ctypes.c_float),
        ("lpf_tf", ctypes.c_float),
    ]

    def __repr__(self):
        fields = {name: getattr(self, name) for name, _ in self._fields_}
        return f"{self.__class__.__name__}({fields})"


class StatusPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("flags", ctypes.c_uint32),
        ("front_left", MotorState),
        ("front_right", MotorState),
        ("rear_left", MotorState),
        ("rear_right", MotorState),
        ("batt_voltage", ctypes.c_float),
        ("batt_current", ctypes.c_float),
        ("batt_percent", ctypes.c_float),
    ]

    def __repr__(self):
        fields = {name: getattr(self, name) for name, _ in self._fields_}
        return f"{self.__class__.__name__}({fields})"


class ImuPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("acc", ctypes.c_float * 3),
        ("gyr", ctypes.c_float * 3),
        ("quat", ctypes.c_float * 4),
    ]

    def __repr__(self):
        return str({"acc": list(self.acc), "gyr": list(self.gyr), "quat": list(self.quat),})


class TofPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("distance_mm", ctypes.c_int16 * 64),
        ("reflectance", ctypes.c_uint8 * 64),
        ("status", ctypes.c_uint8 * 64),
    ]

    def __repr__(self):
        return str({"dist_mm": list(self.distance_mm), "ref": list(self.reflectance), "status": list(self.status)})

class ControlPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("front_left", ctypes.c_float),
        ("front_right", ctypes.c_float),
        ("rear_left", ctypes.c_float),
        ("rear_right", ctypes.c_float),
    ]

    def __repr__(self):
        fields = {name: getattr(self, name) for name, _ in self._fields_}
        return f"{self.__class__.__name__}({fields})"


def read_packet(ser: Serial):
    try:
        frame = ser.read_until(b"\x00")
        frame = frame.rstrip(b"\x00")
        if len(frame) < 3:
            logging.error(f"FRAME TOO SMALL: {frame}")
            return
        frame = cobs.decode(frame)
        type, payload, crc = frame[0], frame[1:-1], frame[-1]
        type = PacketType(type)
        crc_check =  CRC.calc(payload)
        assert crc == crc_check, "crc check failed"
        if type == PacketType.STATUS:
            return type, StatusPacket.from_buffer_copy(payload)
        elif type == PacketType.IMU:
            return type, ImuPacket.from_buffer_copy(payload)
        elif type == PacketType.TOF:
            return type, TofPacket.from_buffer_copy(payload)
    except Exception as err:
        logging.error(f"PROTO ERR: {err}")


def write_packet(ser: Serial, packet):
    if isinstance(packet, ControlPacket):
        msg_type = PacketType.CONTROL
    elif isinstance(packet, PidState):
        msg_type = PacketType.PID
    else:
        raise RuntimeError(f"Unknown packet type: {type(packet)}")

    payload = bytes(packet)
    crc = CRC.calc(payload).to_bytes(1, 'little')
    frame = cobs.encode(msg_type.to_bytes(1, 'little') + payload + crc) + b'\x00'
    ser.write(frame)
    ser.flush()


if __name__ == "__main__":
    from threading import Thread, main_thread
    import time
    
    PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    SPEED = 921600
    ser = Serial(port=PORT, baudrate=SPEED)
    
    def send_speeds(a, b, c, d):
        pack = ControlPacket(float(a), float(-b), float(c), float(-d))
        print(f"SENDING >>> {pack}")
        #write_packet(ser, pack)

    def read_loop():
        while main_thread().is_alive():
            ret = read_packet(ser)
            if ret is not None:
                type, pack = ret
                print (f"RECV <<< {type.name}: {pack}")
    read_thread = Thread(target=read_loop, daemon=True)
    read_thread.start()

    for i in range(4):
        arr = [0] * 4
        arr[i] = 10
        send_speeds(*arr)
        time.sleep(1.0)

    send_speeds(0, 0, 0, 0)
