import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

from F450Dynamics import F450Dynamics


RESULTS_ROOT = "results"
PLOTS_DIR = "plots_f450_pid_optimization"

OPT_DIR = os.path.join(RESULTS_ROOT, "pid_optimized_results")
BASELINE_DIR = os.path.join(RESULTS_ROOT, "pid_baseline_results")

os.makedirs(OPT_DIR, exist_ok=True)
os.makedirs(BASELINE_DIR, exist_ok=True)
os.makedirs(PLOTS_DIR, exist_ok=True)

plt.rcParams.update({
    "font.size": 12,
    "font.family": "serif",
})


def save_plot(name):
    pdf = os.path.join(PLOTS_DIR, f"{name}.pdf")
    png = os.path.join(PLOTS_DIR, f"{name}.png")

    plt.savefig(pdf, dpi=300, bbox_inches="tight")
    plt.savefig(png, dpi=300, bbox_inches="tight")

    print(f"[OK] Saved: {pdf}")
    print(f"[OK] Saved: {png}")


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

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt

        if self.limit_integral is not None:
            self.integral = np.clip(
                self.integral,
                -self.limit_integral,
                self.limit_integral,
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


class CascadedPIDController:
    def __init__(self, gains, dt, env):
        (
            kp_xy, kd_xy,
            kp_z, ki_z, kd_z,
            kp_att, kd_att,
            kp_yaw, kd_yaw,
        ) = gains

        self.env = env
        self.dt = dt

        self.pid_x = PID(kp_xy, 0.0, kd_xy, dt, limit_output=2.5)
        self.pid_y = PID(kp_xy, 0.0, kd_xy, dt, limit_output=2.5)

        self.pid_z = PID(
            kp_z,
            ki_z,
            kd_z,
            dt,
            limit_output=8.0,
            limit_integral=3.0,
        )

        self.pid_phi = PID(kp_att, 0.0, kd_att, dt, limit_output=0.8)
        self.pid_theta = PID(kp_att, 0.0, kd_att, dt, limit_output=0.8)
        self.pid_psi = PID(kp_yaw, 0.0, kd_yaw, dt, limit_output=0.4)

        self.inv_mixer = np.linalg.inv(env.mixer_matrix)
        self.max_angle = np.deg2rad(25.0)

    def compute_action(self, state, ref):
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        x_ref, y_ref, z_ref, psi_ref = ref

        acc_cmd_x = self.pid_x.compute(x_ref, x)
        acc_cmd_y = self.pid_y.compute(y_ref, y)

        phi_ref = (acc_cmd_x * np.sin(psi) - acc_cmd_y * np.cos(psi)) / self.env.g
        theta_ref = (acc_cmd_x * np.cos(psi) + acc_cmd_y * np.sin(psi)) / self.env.g

        phi_ref = np.clip(phi_ref, -self.max_angle, self.max_angle)
        theta_ref = np.clip(theta_ref, -self.max_angle, self.max_angle)

        u_z = self.env.m * self.env.g + self.pid_z.compute(z_ref, z)

        tilt_compensation = np.cos(phi) * np.cos(theta)
        tilt_compensation = np.clip(tilt_compensation, 0.5, 1.0)

        u_z = u_z / tilt_compensation
        u_z = np.clip(
            u_z,
            0.0,
            4.0 * self.env.kf * self.env.omega_max**2,
        )

        u_phi = self.pid_phi.compute(phi_ref, phi)
        u_theta = self.pid_theta.compute(theta_ref, theta)
        u_psi = self.pid_psi.compute(psi_ref, psi)

        command = np.array([u_z, u_phi, u_theta, u_psi])

        w_squared = self.inv_mixer @ command
        w_squared = np.clip(
            w_squared,
            self.env.omega_min**2,
            self.env.omega_max**2,
        )

        motor_action = np.sqrt(w_squared)

        return motor_action, command, phi_ref, theta_ref


def reference_trajectory(t):
    if t < 2.0:
        return np.array([0.0, 0.0, 2.0, 0.0])
    elif t < 6.0:
        return np.array([3.0, 0.0, 2.0, 0.0])
    elif t < 10.0:
        return np.array([3.0, 3.0, 3.0, 0.0])
    else:
        return np.array([0.0, 0.0, 2.0, 0.0])


def estimate_power(env, motor_action):
    if hasattr(env, "estimate_total_current"):
        current = env.estimate_total_current(motor_action)
    else:
        current = 0.0

    if hasattr(env, "effective_battery_voltage"):
        voltage = env.effective_battery_voltage(motor_action)
    else:
        voltage = 11.1

    return voltage * current, voltage, current


def simulate_pid(gains, scenario_id=0, tf=14.0, dt=0.01, save_data=False):
    env = F450Dynamics()
    controller = CascadedPIDController(gains, dt, env)

    steps = int(tf / dt)
    state = env.reset()

    rows = []

    error_history = []
    power_history = []
    controller_time_history = []

    cumulative_energy = 0.0
    cost = 0.0
    unstable = False

    for i in range(steps):
        t = i * dt
        ref = reference_trajectory(t)

        start = time.perf_counter()
        motor_action, command, phi_ref, theta_ref = controller.compute_action(state, ref)
        controller_wall_time_s = time.perf_counter() - start

        state = env.step(motor_action, dt)

        pos = state[0:3]
        att = state[6:9]

        position_error_vec = ref[0:3] - pos
        position_error_m = np.linalg.norm(position_error_vec)
        yaw_error = ref[3] - state[8]

        total_power_W, voltage_V, current_A = estimate_power(env, motor_action)
        cumulative_energy += total_power_W * dt

        error_history.append(position_error_m)
        power_history.append(total_power_W)
        controller_time_history.append(controller_wall_time_s)

        motor_ratio = motor_action / env.omega_max

        step_cost = (
            10.0 * position_error_m**2
            + 0.5 * np.dot(att, att)
            + 0.2 * yaw_error**2
            + 0.05 * np.mean(motor_ratio**2)
        )

        cost += step_cost * dt

        if (
            np.any(np.isnan(state))
            or abs(state[0]) > 25.0
            or abs(state[1]) > 25.0
            or state[2] < -3.0
            or state[2] > 20.0
            or abs(state[6]) > np.deg2rad(80.0)
            or abs(state[7]) > np.deg2rad(80.0)
        ):
            unstable = True
            cost += 1e6
            break

        if save_data:
            rows.append({
                "scenario_id": scenario_id,
                "time_s": t,

                "x_ref_m": ref[0],
                "y_ref_m": ref[1],
                "z_ref_m": ref[2],
                "psi_ref_rad": ref[3],

                "x_m": state[0],
                "y_m": state[1],
                "z_m": state[2],

                "vx_m_s": state[3],
                "vy_m_s": state[4],
                "vz_m_s": state[5],

                "phi_rad": state[6],
                "theta_rad": state[7],
                "psi_rad": state[8],

                "phi_deg": np.rad2deg(state[6]),
                "theta_deg": np.rad2deg(state[7]),
                "psi_deg": np.rad2deg(state[8]),

                "p_rad_s": state[9],
                "q_rad_s": state[10],
                "r_rad_s": state[11],

                "phi_ref_rad": phi_ref,
                "theta_ref_rad": theta_ref,
                "phi_ref_deg": np.rad2deg(phi_ref),
                "theta_ref_deg": np.rad2deg(theta_ref),

                "position_error_x_m": position_error_vec[0],
                "position_error_y_m": position_error_vec[1],
                "position_error_z_m": position_error_vec[2],
                "position_error_m": position_error_m,

                "u_thrust_N": command[0],
                "u_tau_x_Nm": command[1],
                "u_tau_y_Nm": command[2],
                "u_tau_z_Nm": command[3],

                "omega_1_rad_s": motor_action[0],
                "omega_2_rad_s": motor_action[1],
                "omega_3_rad_s": motor_action[2],
                "omega_4_rad_s": motor_action[3],

                "omega_mean_rad_s": np.mean(motor_action),
                "omega_max_rad_s": np.max(motor_action),

                "battery_voltage_V": voltage_V,
                "total_current_A": current_A,
                "total_power_W": total_power_W,
                "cumulative_energy_J": cumulative_energy,

                "controller_wall_time_s": controller_wall_time_s,
            })

    df = pd.DataFrame(rows) if rows else pd.DataFrame()

    if len(error_history) > 0:
        err = np.asarray(error_history)
        power = np.asarray(power_history)
        ctrl_time = np.asarray(controller_time_history)

        rmse = np.sqrt(np.mean(err**2))
        mae = np.mean(np.abs(err))
        final_error = err[-1]
        final_energy = cumulative_energy
        mean_power = np.mean(power)
        mean_controller_time = np.mean(ctrl_time)
    else:
        rmse = np.inf
        mae = np.inf
        final_error = np.inf
        final_energy = np.inf
        mean_power = np.inf
        mean_controller_time = np.inf

    final_ref = reference_trajectory(tf)
    final_position_error = np.linalg.norm(final_ref[0:3] - state[0:3])
    cost += 100.0 * final_position_error**2

    metrics = {
        "scenario_id": scenario_id,
        "cost": cost,
        "rmse": rmse,
        "mae": mae,
        "final_error": final_error,
        "final_energy": final_energy,
        "mean_power": mean_power,
        "mean_controller_time": mean_controller_time,
        "unstable": unstable,
    }

    return cost, df, metrics


def objective(gains):
    cost, _, _ = simulate_pid(
        gains,
        scenario_id=0,
        tf=14.0,
        dt=0.01,
        save_data=False,
    )
    return cost


optimization_history = []


def optimization_callback(xk, convergence):
    generation = len(optimization_history) + 1

    cost, _, metrics = simulate_pid(
        xk,
        scenario_id=0,
        tf=14.0,
        dt=0.01,
        save_data=False,
    )

    row = {
        "generation": generation,
        "cost": cost,
        "rmse": metrics["rmse"],
        "mae": metrics["mae"],
        "final_error": metrics["final_error"],
        "final_energy": metrics["final_energy"],
        "mean_power": metrics["mean_power"],
        "convergence": convergence,
    }

    gain_names = [
        "kp_xy",
        "kd_xy",
        "kp_z",
        "ki_z",
        "kd_z",
        "kp_att",
        "kd_att",
        "kp_yaw",
        "kd_yaw",
    ]

    for name, value in zip(gain_names, xk):
        row[name] = value

    optimization_history.append(row)

    hist_path = os.path.join(OPT_DIR, "pid_optimization_history.csv")
    pd.DataFrame(optimization_history).to_csv(hist_path, index=False)

    print(
        f"[GEN {generation:03d}] "
        f"cost={cost:.4f} | "
        f"RMSE={metrics['rmse']:.4f} m | "
        f"final_error={metrics['final_error']:.4f} m"
    )

    return False


def optimize_pid():
    bounds = [
        (0.05, 2.5),    # kp_xy
        (0.05, 2.5),    # kd_xy

        (2.0, 25.0),    # kp_z
        (0.0, 6.0),     # ki_z
        (1.0, 18.0),    # kd_z

        (0.5, 10.0),    # kp_att
        (0.05, 3.0),    # kd_att

        (0.2, 5.0),     # kp_yaw
        (0.01, 1.5),    # kd_yaw
    ]

    result = differential_evolution(
        objective,
        bounds=bounds,
        strategy="best1bin",
        maxiter=60,
        popsize=12,
        tol=1e-4,
        mutation=(0.5, 1.0),
        recombination=0.7,
        polish=True,
        workers=1,
        updating="immediate",
        seed=42,
        disp=True,
        callback=optimization_callback,
    )

    best_gains = result.x

    gain_names = [
        "kp_xy",
        "kd_xy",
        "kp_z",
        "ki_z",
        "kd_z",
        "kp_att",
        "kd_att",
        "kp_yaw",
        "kd_yaw",
    ]

    gains_df = pd.DataFrame({
        "gain": gain_names,
        "value": best_gains,
    })

    gains_path = os.path.join(OPT_DIR, "pid_optimized_gains.csv")
    gains_df.to_csv(gains_path, index=False)

    print(f"[OK] Saved best gains: {gains_path}")
    print("\nBest cost:", result.fun)
    print(gains_df)

    return best_gains


def compute_metrics_from_df(df):
    rows = []

    for sid, g in df.groupby("scenario_id"):
        err = g["position_error_m"].values
        power = g["total_power_W"].values
        ctrl = g["controller_wall_time_s"].values

        rows.append({
            "scenario_id": sid,
            "rmse": np.sqrt(np.mean(err**2)),
            "mae": np.mean(np.abs(err)),
            "final_error": err[-1],
            "final_energy": g["cumulative_energy_J"].iloc[-1],
            "mean_power": np.mean(power),
            "controller_time": np.mean(ctrl),
            "frequency": 1.0 / np.mean(ctrl) if np.mean(ctrl) > 0 else np.nan,
        })

    return pd.DataFrame(rows)


def save_final_simulation(gains, output_dir, filename, scenario_id=0):
    cost, df, metrics = simulate_pid(
        gains,
        scenario_id=scenario_id,
        tf=14.0,
        dt=0.01,
        save_data=True,
    )

    csv_path = os.path.join(output_dir, filename)
    df.to_csv(csv_path, index=False)

    metrics_df = compute_metrics_from_df(df)
    metrics_path = os.path.join(
        output_dir,
        filename.replace("_data.csv", "_metrics.csv"),
    )
    metrics_df.to_csv(metrics_path, index=False)

    print(f"[OK] Saved simulation data: {csv_path}")
    print(f"[OK] Saved metrics: {metrics_path}")

    return df, metrics_df


def plot_optimization_history():
    path = os.path.join(OPT_DIR, "pid_optimization_history.csv")

    if not os.path.exists(path):
        print(f"[WARNING] Optimization history not found: {path}")
        return

    df = pd.read_csv(path)

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(df["generation"], df["cost"], linewidth=1.8)
    plt.xlabel("Generation")
    plt.ylabel("Cost")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_plot("pid_optimization_cost")

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(df["generation"], df["rmse"], linewidth=1.8)
    plt.xlabel("Generation")
    plt.ylabel("RMSE [m]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_plot("pid_optimization_rmse")

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(df["generation"], df["final_error"], linewidth=1.8)
    plt.xlabel("Generation")
    plt.ylabel("Final Position Error [m]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_plot("pid_optimization_final_error")


def plot_final_results(df, label="PID_optimized", color="#d62728", ls="--"):
    plt.figure(figsize=(6.5, 4.2))
    plt.plot(
        df["time_s"],
        df["position_error_m"],
        label=label,
        color=color,
        linestyle=ls,
        linewidth=1.8,
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(frameon=True)
    plt.tight_layout()
    save_plot("f450_position_error_time")

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(df["time_s"], df["x_m"], label="x", linewidth=1.8)
    plt.plot(df["time_s"], df["y_m"], label="y", linewidth=1.8)
    plt.plot(df["time_s"], df["z_m"], label="z", linewidth=1.8)
    plt.plot(df["time_s"], df["x_ref_m"], "--", label="x_ref", linewidth=1.4)
    plt.plot(df["time_s"], df["y_ref_m"], "--", label="y_ref", linewidth=1.4)
    plt.plot(df["time_s"], df["z_ref_m"], "--", label="z_ref", linewidth=1.4)
    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(frameon=True)
    plt.tight_layout()
    save_plot("f450_position_tracking")

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(df["time_s"], df["phi_deg"], label="Roll", linewidth=1.8)
    plt.plot(df["time_s"], df["theta_deg"], label="Pitch", linewidth=1.8)
    plt.plot(df["time_s"], df["psi_deg"], label="Yaw", linewidth=1.8)
    plt.xlabel("Time [s]")
    plt.ylabel("Attitude [deg]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(frameon=True)
    plt.tight_layout()
    save_plot("f450_attitude_time")

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(
        df["time_s"],
        df["total_power_W"],
        color=color,
        linestyle=ls,
        linewidth=1.8,
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Power [W]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_plot("f450_power_time")

    plt.figure(figsize=(6.5, 4.2))
    plt.plot(
        df["time_s"],
        df["cumulative_energy_J"],
        color=color,
        linestyle=ls,
        linewidth=1.8,
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Energy [J]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_plot("f450_energy_time")


def main():
    baseline_gains = np.array([
        0.45, 0.80,
        8.0, 1.5, 5.0,
        1.8, 0.35,
        0.8, 0.15,
    ])

    print("\n[INFO] Saving baseline simulation...")
    baseline_df, baseline_metrics = save_final_simulation(
        baseline_gains,
        BASELINE_DIR,
        "pid_baseline_f450_position_data.csv",
        scenario_id=0,
    )

    print("\n[INFO] Starting PID optimization...")
    best_gains = optimize_pid()

    print("\n[INFO] Saving optimized simulation...")
    optimized_df, optimized_metrics = save_final_simulation(
        best_gains,
        OPT_DIR,
        "pid_optimized_f450_position_data.csv",
        scenario_id=0,
    )

    print("\n[INFO] Plotting optimization history...")
    plot_optimization_history()

    print("\n[INFO] Plotting final optimized results...")
    plot_final_results(optimized_df)

    print("\n=== BASELINE METRICS ===")
    print(baseline_metrics)

    print("\n=== OPTIMIZED METRICS ===")
    print(optimized_metrics)

    plt.show()


if __name__ == "__main__":
    main()