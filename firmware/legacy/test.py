import struct
import serial
from dataclasses import dataclass
from cobs import cobs

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = 921600
MSG_INA = 1


@dataclass
class InaPayload:
    volt_mV: int
    curr_mA: int
    soc_x100: int
    time_ms: int

    STRUCT = struct.Struct("<iiHI")

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) != cls.STRUCT.size:
            raise ValueError(f"bad INA payload size: {len(data)}")

        return cls(*cls.STRUCT.unpack(data))

    @property
    def voltage_v(self):
        return self.volt_mV / 1000.0

    @property
    def soc(self):
        return self.soc_x100 / 100.0


uart = serial.Serial(PORT, BAUD, timeout=0.1)
buf = bytearray()

print("waiting packets...")

while True:
    data = uart.read(128)

    if data:
        buf.extend(data)

    while 0 in buf:
        end = buf.index(0)
        raw = bytes(buf[:end])
        del buf[:end + 1]

        if not raw:
            continue

        try:
            decoded = cobs.decode(raw)
        except Exception as e:
            print("bad cobs:", raw.hex(" "), e)
            continue

        if len(decoded) < 1:
            continue

        msg_type = decoded[0]
        payload = decoded[1:]

        if msg_type != MSG_INA:
            print("unknown msg:", msg_type)
            continue

        try:
            ina = InaPayload.from_bytes(payload)
        except ValueError as e:
            print(e)
            print("decoded:", decoded.hex(" "))
            continue

        print(
            f"V={ina.voltage_v:.3f}V "
            f"I={ina.curr_mA}mA "
            f"SOC={ina.soc:.2f}% "
            f"t={ina.time_ms}ms"
        )
