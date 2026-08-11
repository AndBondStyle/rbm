import os
import struct
import serial
from dataclasses import dataclass
from cobs import cobs


PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = 921600

OUT_FILE = "telemetry.txt"

MSG_INA = 1
MSG_TOF = 2
MSG_IMU = 3


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

    def to_line(self) -> str:
        return (
            f"INA: "
            f"V={self.volt_mV / 1000.0:.3f}V "
            f"I={self.curr_mA}mA "
            f"SOC={self.soc_x100 / 100.0:.2f}% "
            f"t={self.time_ms}ms"
        )


@dataclass
class ImuPayload:
    acc: list
    gyr: list
    mag: list
    quat: list
    time_ms: int

    STRUCT = struct.Struct("<3h3h3h4hI")

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) != cls.STRUCT.size:
            raise ValueError(f"bad IMU payload size: {len(data)}")

        values = cls.STRUCT.unpack(data)

        acc = list(values[0:3])
        gyr = list(values[3:6])
        mag = list(values[6:9])
        quat = list(values[9:13])
        time_ms = values[13]

        return cls(acc, gyr, mag, quat, time_ms)

    def to_line(self) -> str:
        acc_g = [x / 1000.0 for x in self.acc]
        gyr_dps = [x / 10.0 for x in self.gyr]
        mag_uT = [x / 10.0 for x in self.mag]
        quat = [x / 32767.0 for x in self.quat]

        return (
            f"IMU: "
            f"acc={acc_g}g "
            f"gyr={gyr_dps}dps "
            f"mag={mag_uT}uT "
            f"quat={quat} "
            f"t={self.time_ms}ms"
        )


TOF_ZONES = 64

@dataclass
class TofPayload:
    zones: int
    distance_mm: list
    status: list
    time_ms: int

    STRUCT = struct.Struct("<B64h64BI")

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) != cls.STRUCT.size:
            raise ValueError(f"bad TOF payload size: {len(data)}")

        values = cls.STRUCT.unpack(data)

        zones = values[0]
        distance_mm = list(values[1:1 + TOF_ZONES])
        status = list(values[1 + TOF_ZONES:1 + TOF_ZONES + TOF_ZONES])
        time_ms = values[1 + TOF_ZONES + TOF_ZONES]

        return cls(zones, distance_mm, status, time_ms)

    def to_line(self) -> str:
        rows = []
        for y in range(8):
            row = self.distance_mm[y * 8:(y + 1) * 8]
            rows.append(row)
        return (
            f"TOF: "
            f"zones={self.zones} "
            f"dist={rows} "
            f"status={self.status} "
            f"t={self.time_ms}ms"
        )


class TelemetryFile:
    def __init__(self, path: str):
        self.path = path

        self.lines = {
            MSG_INA: "INA: no data",
            MSG_IMU: "IMU: no data",
            MSG_TOF: "TOF: no data",
        }

    def update(self, msg_type: int, line: str):
        self.lines[msg_type] = line
        self.write_file()

    def write_file(self):
        tmp_path = self.path + ".tmp"

        with open(tmp_path, "w", encoding="utf-8") as f:
            f.write(self.lines[MSG_INA] + "\n")
            f.write(self.lines[MSG_IMU] + "\n")
            f.write(self.lines[MSG_TOF] + "\n")

        os.replace(tmp_path, self.path)


def parse_packet(decoded: bytes):
    if len(decoded) < 1:
        return None, None

    msg_type = decoded[0]
    payload = decoded[1:]

    if msg_type == MSG_INA:
        return msg_type, InaPayload.from_bytes(payload).to_line()

    if msg_type == MSG_IMU:
        return msg_type, ImuPayload.from_bytes(payload).to_line()

    if msg_type == MSG_TOF:
        return msg_type, TofPayload.from_bytes(payload).to_line()

    return None, None


def main():
    uart = serial.Serial(PORT, BAUD, timeout=0.1)
    buf = bytearray()
    telemetry = TelemetryFile(OUT_FILE)

    print(f"writing telemetry to {OUT_FILE}")

    while True:
        data = uart.read(256)

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
            except Exception:
                continue

            try:
                msg_type, line = parse_packet(decoded)
            except ValueError:
                continue

            if msg_type is not None and line is not None:
                telemetry.update(msg_type, line)


if __name__ == "__main__":
    main()
