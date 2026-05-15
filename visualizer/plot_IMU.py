import serial
import time
import math
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


SERIAL_PORT = "/dev/cu.usbserial-130"
BAUD_RATE = 115200

MAX_POINTS = 300
ALPHA = 0.98  # filtro complementar


def parse_imu_line(line: str):
    """
    Expected format:
    <IMU,109871,0.2068,0.0396,0.9658,0.9084,-2.3740,-0.9237>
    """
    line = line.strip()

    if not line.startswith("<") or not line.endswith(">"):
        return None

    line = line[1:-1]
    parts = line.split(",")

    if len(parts) != 8:
        return None

    if parts[0] != "IMU":
        return None

    try:
        timestamp_ms = float(parts[1])
        ax = float(parts[2])
        ay = float(parts[3])
        az = float(parts[4])
        gx = float(parts[5])
        gy = float(parts[6])
        gz = float(parts[7])
    except ValueError:
        return None

    return timestamp_ms, ax, ay, az, gx, gy, gz


class IMUOrientationEstimator:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        self.initialized = False

    def update(self, timestamp_ms, ax, ay, az, gx, gy, gz):
        current_time = timestamp_ms / 1000.0

        if self.last_time is None:
            self.last_time = current_time

        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0 or dt > 1.0:
            dt = 0.01

        # Roll e pitch pelo acelerômetro
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        if not self.initialized:
            self.roll = roll_acc
            self.pitch = pitch_acc
            self.yaw = 0.0
            self.initialized = True
            return self.roll, self.pitch, self.yaw

        # Integração do giroscópio
        roll_gyro = self.roll + gx * dt
        pitch_gyro = self.pitch + gy * dt
        yaw_gyro = self.yaw + gz * dt

        # Filtro complementar
        self.roll = ALPHA * roll_gyro + (1.0 - ALPHA) * roll_acc
        self.pitch = ALPHA * pitch_gyro + (1.0 - ALPHA) * pitch_acc
        self.yaw = yaw_gyro

        return self.roll, self.pitch, self.yaw


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    estimator = IMUOrientationEstimator()

    t_data = deque(maxlen=MAX_POINTS)

    ax_data = deque(maxlen=MAX_POINTS)
    ay_data = deque(maxlen=MAX_POINTS)
    az_data = deque(maxlen=MAX_POINTS)

    gx_data = deque(maxlen=MAX_POINTS)
    gy_data = deque(maxlen=MAX_POINTS)
    gz_data = deque(maxlen=MAX_POINTS)

    roll_data = deque(maxlen=MAX_POINTS)
    pitch_data = deque(maxlen=MAX_POINTS)
    yaw_data = deque(maxlen=MAX_POINTS)

    start_time = None

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax_acc, ax_gyro, ax_ori = axes

    acc_x_line, = ax_acc.plot([], [], label="ax")
    acc_y_line, = ax_acc.plot([], [], label="ay")
    acc_z_line, = ax_acc.plot([], [], label="az")

    gyro_x_line, = ax_gyro.plot([], [], label="gx")
    gyro_y_line, = ax_gyro.plot([], [], label="gy")
    gyro_z_line, = ax_gyro.plot([], [], label="gz")

    roll_line, = ax_ori.plot([], [], label="roll")
    pitch_line, = ax_ori.plot([], [], label="pitch")
    yaw_line, = ax_ori.plot([], [], label="yaw")

    ax_acc.set_ylabel("Acceleration [g]")
    ax_gyro.set_ylabel("Gyroscope [deg/s]")
    ax_ori.set_ylabel("Orientation [deg]")
    ax_ori.set_xlabel("Time [s]")

    ax_acc.grid(True)
    ax_gyro.grid(True)
    ax_ori.grid(True)

    ax_acc.legend(loc="upper right")
    ax_gyro.legend(loc="upper right")
    ax_ori.legend(loc="upper right")

    def update_plot(frame):
        nonlocal start_time

        while ser.in_waiting:
            raw_line = ser.readline().decode("utf-8", errors="ignore")
            parsed = parse_imu_line(raw_line)

            if parsed is None:
                continue

            timestamp_ms, ax, ay, az, gx, gy, gz = parsed

            if start_time is None:
                start_time = timestamp_ms

            t = (timestamp_ms - start_time) / 1000.0

            roll, pitch, yaw = estimator.update(
                timestamp_ms, ax, ay, az, gx, gy, gz
            )

            t_data.append(t)

            ax_data.append(ax)
            ay_data.append(ay)
            az_data.append(az)

            gx_data.append(gx)
            gy_data.append(gy)
            gz_data.append(gz)

            roll_data.append(roll)
            pitch_data.append(pitch)
            yaw_data.append(yaw)

        if len(t_data) == 0:
            return []

        acc_x_line.set_data(t_data, ax_data)
        acc_y_line.set_data(t_data, ay_data)
        acc_z_line.set_data(t_data, az_data)

        gyro_x_line.set_data(t_data, gx_data)
        gyro_y_line.set_data(t_data, gy_data)
        gyro_z_line.set_data(t_data, gz_data)

        roll_line.set_data(t_data, roll_data)
        pitch_line.set_data(t_data, pitch_data)
        yaw_line.set_data(t_data, yaw_data)

        for axis in axes:
            axis.relim()
            axis.autoscale_view()

        return [
            acc_x_line, acc_y_line, acc_z_line,
            gyro_x_line, gyro_y_line, gyro_z_line,
            roll_line, pitch_line, yaw_line
        ]

    ani = FuncAnimation(
        fig,
        update_plot,
        interval=20,
        blit=False
    )

    plt.tight_layout()
    plt.show()

    ser.close()


if __name__ == "__main__":
    main()