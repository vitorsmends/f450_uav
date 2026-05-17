import numpy as np
import matplotlib.pyplot as plt
from F450Dynamics import F450Dynamics


class PID:
    def __init__(self, kp, ki, kd, dt, limit_output=None, limit_integral=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit_output = limit_output
        self.limit_integral = limit_integral
        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement

        self.integral += error * self.dt

        if self.limit_integral is not None:
            self.integral = np.clip(
                self.integral,
                -self.limit_integral,
                self.limit_integral
            )

        derivative = (error - self.previous_error) / self.dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        if self.limit_output is not None:
            output = np.clip(output, -self.limit_output, self.limit_output)

        self.previous_error = error
        return output


def run_simulation():
    env = F450Dynamics()

    dt = 0.01
    tf = 10.0
    steps = int(tf / dt)

    inv_mixer = np.linalg.inv(env.mixer_matrix)

    # Outer-loop position controllers
    pid_x = PID(kp=0.45, ki=0.0, kd=0.80, dt=dt, limit_output=2.5)
    pid_y = PID(kp=0.45, ki=0.0, kd=0.80, dt=dt, limit_output=2.5)
    pid_z = PID(kp=8.0, ki=1.5, kd=5.0, dt=dt, limit_output=8.0, limit_integral=3.0)

    # Inner-loop attitude controllers
    pid_phi = PID(kp=1.8, ki=0.0, kd=0.35, dt=dt, limit_output=0.8)
    pid_theta = PID(kp=1.8, ki=0.0, kd=0.35, dt=dt, limit_output=0.8)
    pid_psi = PID(kp=0.8, ki=0.0, kd=0.15, dt=dt, limit_output=0.4)

    x_ref = 5.0
    y_ref = 5.0
    z_ref = 5.0
    psi_ref = 0.0

    max_angle = np.deg2rad(25.0)

    history = {
        "t": [],
        "x": [],
        "y": [],
        "z": [],
        "phi": [],
        "theta": [],
        "psi": [],
        "omega1": [],
        "omega2": [],
        "omega3": [],
        "omega4": [],
    }

    state = env.reset()

    for i in range(steps):
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state

        # --------------------------------------------------------------
        # Outer loop: position -> desired acceleration
        # --------------------------------------------------------------
        acc_cmd_x = pid_x.compute(x_ref, x)
        acc_cmd_y = pid_y.compute(y_ref, y)

        # Small-angle mapping from desired horizontal acceleration to attitude
        # NED/ENU convention must be checked with the dynamic model.
        phi_ref = (acc_cmd_x * np.sin(psi) - acc_cmd_y * np.cos(psi)) / env.g
        theta_ref = (acc_cmd_x * np.cos(psi) + acc_cmd_y * np.sin(psi)) / env.g

        phi_ref = np.clip(phi_ref, -max_angle, max_angle)
        theta_ref = np.clip(theta_ref, -max_angle, max_angle)

        # --------------------------------------------------------------
        # Altitude control
        # --------------------------------------------------------------
        u_z = env.m * env.g + pid_z.compute(z_ref, z)

        # Compensate thrust loss when the drone tilts
        tilt_compensation = np.cos(phi) * np.cos(theta)
        tilt_compensation = np.clip(tilt_compensation, 0.5, 1.0)

        u_z = u_z / tilt_compensation
        u_z = np.clip(u_z, 0.0, 4.0 * env.kf * env.omega_max**2)

        # --------------------------------------------------------------
        # Inner loop: attitude control
        # --------------------------------------------------------------
        u_phi = pid_phi.compute(phi_ref, phi)
        u_theta = pid_theta.compute(theta_ref, theta)
        u_psi = pid_psi.compute(psi_ref, psi)

        command = np.array([u_z, u_phi, u_theta, u_psi])

        # --------------------------------------------------------------
        # Control allocation
        # --------------------------------------------------------------
        w_squared = inv_mixer @ command

        w_squared = np.clip(
            w_squared,
            env.omega_min**2,
            env.omega_max**2
        )

        motor_action = np.sqrt(w_squared)

        # --------------------------------------------------------------
        # Simulation step
        # --------------------------------------------------------------
        state = env.step(motor_action, dt)

        history["t"].append(i * dt)
        history["x"].append(state[0])
        history["y"].append(state[1])
        history["z"].append(state[2])
        history["phi"].append(np.rad2deg(state[6]))
        history["theta"].append(np.rad2deg(state[7]))
        history["psi"].append(np.rad2deg(state[8]))
        history["omega1"].append(motor_action[0])
        history["omega2"].append(motor_action[1])
        history["omega3"].append(motor_action[2])
        history["omega4"].append(motor_action[3])

    plot_results(history, x_ref, y_ref, z_ref)


def plot_results(history, x_ref, y_ref, z_ref):
    t = history["t"]

    plt.figure(figsize=(10, 10))

    plt.subplot(3, 1, 1)
    plt.plot(t, history["x"], label="Position X")
    plt.axhline(y=x_ref, linestyle="--", label="Reference X")
    plt.ylabel("X [m]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, history["y"], label="Position Y")
    plt.axhline(y=y_ref, linestyle="--", label="Reference Y")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, history["z"], label="Altitude Z")
    plt.axhline(y=z_ref, linestyle="--", label="Reference Z")
    plt.xlabel("Time [s]")
    plt.ylabel("Z [m]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(t, history["phi"], label="Roll")
    plt.plot(t, history["theta"], label="Pitch")
    plt.plot(t, history["psi"], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(t, history["omega1"], label="Motor 1")
    plt.plot(t, history["omega2"], label="Motor 2")
    plt.plot(t, history["omega3"], label="Motor 3")
    plt.plot(t, history["omega4"], label="Motor 4")
    plt.xlabel("Time [s]")
    plt.ylabel("Motor speed [rad/s]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_simulation()