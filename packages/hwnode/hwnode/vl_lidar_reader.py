# https://github.com/ferrolho/VL53L5CX-BNO08X-viewer

from serial import Serial
import numpy as np
import threading
import json

PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SPEED = 921600

FOV_DIAGONAL_DEG = 65.0
RESOLUTION = 8
NUM_ZONES = RESOLUTION ** 2
MIN_RANGE_MM = 20


def compute_zone_angles():
    """
    Pre-compute the angle for each zone center.
    The sensor lens flips the image, so zone 0 corresponds to top-right.
    Computes values for both uniform grid and ST lookup table methods.
    """
    # === Uniform Grid Method ===
    # Convert diagonal FoV to per-axis FoV (assuming square sensor)
    # For a square, diagonal = side * sqrt(2), so side = diagonal / sqrt(2)
    fov_per_axis_deg = FOV_DIAGONAL_DEG / np.sqrt(2)
    fov_per_axis_rad = np.deg2rad(fov_per_axis_deg)

    # Angle step per zone
    angle_step = fov_per_axis_rad / RESOLUTION

    # Zone center offsets from optical axis
    # Zones are numbered row-major: 0-7 = row 0, 8-15 = row 1, etc.
    # Due to lens flip, we invert the mapping
    zone_angles_x = np.zeros(NUM_ZONES)
    zone_angles_y = np.zeros(NUM_ZONES)

    for i in range(NUM_ZONES):
        row = i // RESOLUTION
        col = i % RESOLUTION

        # Center of zone relative to center of grid (0-7 -> -3.5 to 3.5)
        # Flip due to lens inversion
        col_offset = (RESOLUTION - 1) / 2 - col  # Flip X
        row_offset = (RESOLUTION - 1) / 2 - row  # Flip Y

        zone_angles_x[i] = col_offset * angle_step
        zone_angles_y[i] = row_offset * angle_step

    # Precompute tan of zone angles for XY calculation
    # The sensor reports perpendicular (z-axis) distance, not radial
    tan_x = np.tan(zone_angles_x)
    tan_y = np.tan(zone_angles_y)
    return tan_x, tan_y


def distances_to_points(distances: np.ndarray, tan_x, tan_y) -> np.ndarray:
    """
    Convert distance measurements to 3D point coordinates.

    The sensor is assumed to be pointing UP (+Z direction),
    lying flat on a horizontal surface. The VL53L5CX reports
    perpendicular (z-axis) distance, not radial distance - the
    chip performs this conversion internally.

    Args:
        distances: Array of 64 distance values in mm (perpendicular)
        zone_angles: Pre-computed zone angle data

    Returns:
        Nx3 array of (x, y, z) coordinates in meters
    """
    # Convert to meters
    z_m = distances / 1000.0

    # Uniform grid method: assumes uniform angular spacing
    # z IS the perpendicular distance, use tangent for lateral offset
    x = z_m * tan_x
    y = z_m * tan_y
    z = z_m

    return np.column_stack([x, y, z])


class LidarReader:
    def __init__(self) -> None:
        self.ser = Serial(PORT, SPEED)
        self.ser.reset_input_buffer()
        self.tan_x, self.tan_y = compute_zone_angles()

        self.received = threading.Event()
        self.stop_event = threading.Event()
        self.distances: np.ndarray = None
        self.statuses: np.ndarray = None
        self.callback = None

        self.serial_thread = threading.Thread(target=self.serial_loop)
        self.process_thread = threading.Thread(target=self.process_loop)

    def serial_loop(self):
        while not self.stop_event.is_set():
            line = self.ser.readline()
            try: data = json.loads(line.decode("utf-8", errors="ignore"))
            except json.JSONDecodeError: continue
            self.distances = np.array(data["distances"], dtype=np.float32)
            self.statuses = np.array(data["status"], dtype=np.uint8)
            self.received.set()

    def process_loop(self):
        while not self.stop_event.is_set():
            self.received.wait(1.0)
            if not self.received.is_set(): continue
            self.received.clear()
            assert self.distances is not None, self.statuses is not None
            distances, status = self.distances.copy(), self.statuses.copy()
            points = distances_to_points(distances, self.tan_x, self.tan_y)
            mask = (status == 5) & (distances >= MIN_RANGE_MM)
            if self.callback is not None:
                self.callback(points, status, mask)

    def start(self):
        self.process_thread.start()
        self.serial_thread.start()

    def stop(self):
        self.stop_event.set()
        self.process_thread.join()
        self.serial_thread.join()


if __name__ == "__main__":
    reader = LidarReader()
    input()
    reader.stop()