import serial
import time
import math
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


SERIAL_PORT = "/dev/tty.usbserial-130"
BAUD_RATE = 115200

MAX_POINTS = 300


def parse_imu_line(line: str):
    """
    Expected format:
    <IMU,109871,0.2068,0.0396,0.9658,0.9084,-2.3740,-0.9237>

    Fields:
    IMU, timestamp_ms, ax, ay, az, gx, gy, gz
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


class KalmanAngle:
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0

        self.P = [
            [0.0, 0.0],
            [0.0, 0.0]
        ]

        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

    def update(self, measured_angle, measured_rate, dt):
        rate = measured_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (
            dt * self.P[1][1]
            - self.P[0][1]
            - self.P[1][0]
            + self.Q_angle
        )
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure

        if S == 0:
            return self.angle

        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        y = measured_angle - self.angle

        self.angle += K0 * y
        self.bias += K1 * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K0 * P00_temp
        self.P[0][1] -= K0 * P01_temp
        self.P[1][0] -= K1 * P00_temp
        self.P[1][1] -= K1 * P01_temp

        return self.angle


class IMUOrientationEstimator:
    def __init__(self):
        self.kalman_roll = KalmanAngle()
        self.kalman_pitch = KalmanAngle()

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

        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(
            math.atan2(-ax, math.sqrt(ay * ay + az * az))
        )

        if not self.initialized:
            self.roll = roll_acc
            self.pitch = pitch_acc
            self.yaw = 0.0

            self.kalman_roll.angle = roll_acc
            self.kalman_pitch.angle = pitch_acc

            self.initialized = True

            return self.roll, self.pitch, self.yaw

        self.roll = self.kalman_roll.update(
            measured_angle=roll_acc,
            measured_rate=gx,
            dt=dt
        )

        self.pitch = self.kalman_pitch.update(
            measured_angle=pitch_acc,
            measured_rate=gy,
            dt=dt
        )

        self.yaw += gz * dt

        return self.roll, self.pitch, self.yaw


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as error:
        print(f"Failed to open serial port: {error}")
        return

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
                timestamp_ms,
                ax,
                ay,
                az,
                gx,
                gy,
                gz
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
            acc_x_line,
            acc_y_line,
            acc_z_line,
            gyro_x_line,
            gyro_y_line,
            gyro_z_line,
            roll_line,
            pitch_line,
            yaw_line
        ]

    try:
        FuncAnimation(
            fig,
            update_plot,
            interval=20,
            blit=False,
            cache_frame_data=False
        )

        plt.tight_layout()
        plt.show()

    finally:
        ser.close()


if __name__ == "__main__":
    main()