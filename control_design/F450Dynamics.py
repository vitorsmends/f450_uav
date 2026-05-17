import numpy as np


class F450Dynamics:
    def __init__(self):
        self.g = 9.81
        self.rho = 1.225  # Air density [kg/m^3]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.m = 1.25653

        self.I = np.array([
            [0.012363,     3.3165e-05,   5.60471e-05],
            [3.3165e-05,   0.0124502,   -0.00031169],
            [5.60471e-05, -0.00031169,   0.0213427]
        ])
        self.inv_I = np.linalg.inv(self.I)

        self.Lx = 0.1589
        self.Ly = 0.1589

        self.omega_max = 827.0  # rad/s
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.omega_min = 0.0

        self.kf = 1.00e-5
        # Estimated thrust coefficient [N.s^2/rad^2]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.km = self.kf * 0.012
        # Estimated yaw moment coefficient [N.m.s^2/rad^2]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.motor_tau_up = 0.08
        self.motor_tau_down = 0.12
        # Estimated motor/ESC time constants [s]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.mixer_matrix = np.array([
            [ self.kf,          self.kf,          self.kf,          self.kf],
            [ self.Ly*self.kf,  self.Ly*self.kf, -self.Ly*self.kf, -self.Ly*self.kf],
            [-self.Lx*self.kf,  self.Lx*self.kf,  self.Lx*self.kf, -self.Lx*self.kf],
            [ self.km,         -self.km,          self.km,         -self.km]
        ])

        self.use_motor_dynamics = True
        self.use_linear_drag = True
        self.use_quadratic_drag = True
        self.use_rotational_drag = True
        self.use_wind = True
        self.use_battery_model = True

        self.linear_drag_body = np.diag([0.10, 0.10, 0.15])
        # Estimated linear drag [N/(m/s)]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.CdA = np.array([0.035, 0.035, 0.060])
        # Estimated equivalent drag area Cd*A [m^2]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.rotational_drag_body = np.diag([0.0020, 0.0020, 0.0030])
        # Estimated angular damping [N.m/(rad/s)]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.wind_inertial = np.array([0.0, 0.0, 0.0])
        # Constant wind velocity [m/s]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.battery_voltage_nominal = 11.1
        self.battery_voltage_full = 12.6
        self.battery_voltage_min = 10.5

        self.battery_internal_resistance = 0.030
        # Estimated total 3S pack internal resistance [ohm]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.current_hover_total = 18.0
        self.current_max_total = 60.0
        # Estimated total current limits [A]
        # TODO_REPLACE_WITH_MEASURED_VALUE

        self.state = np.zeros(12)
        self.motor_omega = np.zeros(4)

    def reset(self, state=None):
        if state is None:
            self.state = np.zeros(12)
        else:
            self.state = np.asarray(state, dtype=float).copy()

        self.motor_omega = np.zeros(4)
        return self.state

    def hover_omega(self):
        return np.sqrt((self.m * self.g / 4.0) / self.kf)

    def estimate_total_current(self, omega):
        omega = np.asarray(omega, dtype=float)
        omega_h = self.hover_omega()

        mean_ratio = np.mean((omega / self.omega_max) ** 2)
        hover_ratio = (omega_h / self.omega_max) ** 2

        if mean_ratio <= hover_ratio:
            current = self.current_hover_total * mean_ratio / max(hover_ratio, 1e-6)
        else:
            alpha = (mean_ratio - hover_ratio) / max(1.0 - hover_ratio, 1e-6)
            current = self.current_hover_total + alpha * (
                self.current_max_total - self.current_hover_total
            )

        return float(np.clip(current, 0.0, self.current_max_total))

    def effective_battery_voltage(self, omega):
        if not self.use_battery_model:
            return self.battery_voltage_nominal

        current = self.estimate_total_current(omega)
        voltage = self.battery_voltage_nominal - current * self.battery_internal_resistance

        return float(np.clip(
            voltage,
            self.battery_voltage_min,
            self.battery_voltage_full
        ))

    def voltage_thrust_scale(self, omega):
        if not self.use_battery_model:
            return 1.0

        v_eff = self.effective_battery_voltage(omega)
        return (v_eff / self.battery_voltage_nominal) ** 2

    def get_rotation_matrix(self, phi, theta, psi):
        cphi, sphi = np.cos(phi), np.sin(phi)
        cthe, sthe = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        return np.array([
            [cthe*cpsi, sphi*sthe*cpsi - cphi*spsi, cphi*sthe*cpsi + sphi*spsi],
            [cthe*spsi, sphi*sthe*spsi + cphi*cpsi, cphi*sthe*spsi - sphi*cpsi],
            [-sthe,     sphi*cthe,                  cphi*cthe]
        ])

    def get_euler_rate_matrix(self, phi, theta):
        cphi, sphi = np.cos(phi), np.sin(phi)
        cthe = np.cos(theta)

        if abs(cthe) < 1e-6:
            cthe = np.sign(cthe) * 1e-6 if cthe != 0 else 1e-6

        tthe = np.sin(theta) / cthe

        return np.array([
            [1.0, sphi*tthe, cphi*tthe],
            [0.0, cphi,      -sphi],
            [0.0, sphi/cthe, cphi/cthe]
        ])

    def update_motor_dynamics(self, omega_cmd, dt):
        omega_cmd = np.clip(
            np.asarray(omega_cmd, dtype=float),
            self.omega_min,
            self.omega_max
        )

        if not self.use_motor_dynamics:
            self.motor_omega = omega_cmd.copy()
            return self.motor_omega

        tau = np.where(
            omega_cmd >= self.motor_omega,
            self.motor_tau_up,
            self.motor_tau_down
        )

        omega_dot = (omega_cmd - self.motor_omega) / tau
        self.motor_omega += dt * omega_dot
        self.motor_omega = np.clip(self.motor_omega, self.omega_min, self.omega_max)

        return self.motor_omega

    def motor_to_generalized_forces(self, omega):
        omega = np.clip(
            np.asarray(omega, dtype=float),
            self.omega_min,
            self.omega_max
        )

        omega_sq = omega ** 2
        U = self.mixer_matrix @ omega_sq

        thrust_scale = self.voltage_thrust_scale(omega)
        U[0:3] *= thrust_scale

        return U

    def aerodynamic_drag(self, vel_inertial, phi, theta, psi):
        R = self.get_rotation_matrix(phi, theta, psi)

        if self.use_wind:
            relative_vel_inertial = vel_inertial - self.wind_inertial
        else:
            relative_vel_inertial = vel_inertial

        relative_vel_body = R.T @ relative_vel_inertial

        F_drag_body = np.zeros(3)

        if self.use_linear_drag:
            F_drag_body += -self.linear_drag_body @ relative_vel_body

        if self.use_quadratic_drag:
            F_drag_body += (
                -0.5
                * self.rho
                * self.CdA
                * relative_vel_body
                * np.abs(relative_vel_body)
            )

        return R @ F_drag_body

    def rotational_drag(self, body_rates):
        if not self.use_rotational_drag:
            return np.zeros(3)

        return -self.rotational_drag_body @ body_rates

    def dynamics(self, state, motor_omega):
        state = np.asarray(state, dtype=float)

        _, _, _, vx, vy, vz, phi, theta, psi, p, q, r = state

        U = self.motor_to_generalized_forces(motor_omega)

        total_thrust = U[0]
        tau = np.array([U[1], U[2], U[3]])

        R = self.get_rotation_matrix(phi, theta, psi)

        F_body = np.array([0.0, 0.0, total_thrust])
        vel_inertial = np.array([vx, vy, vz])

        F_drag_inertial = self.aerodynamic_drag(
            vel_inertial,
            phi,
            theta,
            psi
        )

        gravity = np.array([0.0, 0.0, -self.g])
        linear_acc = gravity + (R @ F_body + F_drag_inertial) / self.m

        body_rates = np.array([p, q, r])
        tau_total = tau + self.rotational_drag(body_rates)

        angular_acc = self.inv_I @ (
            tau_total - np.cross(body_rates, self.I @ body_rates)
        )

        euler_dot = self.get_euler_rate_matrix(phi, theta) @ body_rates

        dx = np.zeros(12)
        dx[0:3] = vel_inertial
        dx[3:6] = linear_acc
        dx[6:9] = euler_dot
        dx[9:12] = angular_acc

        return dx

    def step(self, action, dt=0.01):
        motor_omega = self.update_motor_dynamics(action, dt)

        k1 = self.dynamics(self.state, motor_omega)
        k2 = self.dynamics(self.state + 0.5 * dt * k1, motor_omega)
        k3 = self.dynamics(self.state + 0.5 * dt * k2, motor_omega)
        k4 = self.dynamics(self.state + dt * k3, motor_omega)

        self.state += (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        return self.state

    def check_hover(self):
        omega_h = self.hover_omega()
        rpm_h = omega_h * 60.0 / (2.0 * np.pi)
        U_hover = self.motor_to_generalized_forces(np.full(4, omega_h))

        print("Hover angular speed [rad/s]:", omega_h)
        print("Hover speed [rpm]:", rpm_h)
        print("Generalized forces at hover [T, tau_x, tau_y, tau_z]:")
        print(U_hover)
        print("Vehicle weight [N]:", self.m * self.g)
        print("Estimated loaded voltage at hover [V]:",
              self.effective_battery_voltage(np.full(4, omega_h)))
        print("Estimated total hover current [A]:",
              self.estimate_total_current(np.full(4, omega_h)))

