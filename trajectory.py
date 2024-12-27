import math

class TrajectoryGenerator:
    def __init__(self, max_vel, max_acc, timestamp):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.ts = timestamp

    def triangle_traj(self, pos_init, pos_final, target_time=None):
        delta_target = abs(pos_final - pos_init)
        move_sign = 1 if pos_final >= pos_init else -1

        if target_time is None:
            T_pico = math.sqrt(delta_target / self.max_acc)
            new_amax = self.max_acc
        else:
            new_vmax = (2 * delta_target) / target_time
            new_amax = 2 * new_vmax / target_time
            T_pico = math.sqrt(delta_target / new_amax)

        trajectory = [pos_init]
        current_time = 0.0
        current_vel = 0.0

        while current_time < T_pico:
            current_vel += new_amax * self.ts
            current_vel = min(current_vel, self.max_vel)
            trajectory.append(current_vel * move_sign * self.ts + trajectory[-1])
            current_time += self.ts

        while current_time < 2 * T_pico:
            current_vel -= new_amax * self.ts
            trajectory.append(current_vel * move_sign * self.ts + trajectory[-1])
            current_time += self.ts

        return trajectory, 2 * T_pico

    def trapezoid_traj(self, pos_init, pos_final, target_time=None):
        delta_target = abs(pos_final - pos_init)
        move_sign = 1 if pos_final >= pos_init else -1

        if target_time == None:
            T_pico = self.max_vel / self.max_acc
            T_total = (delta_target + self.max_vel * T_pico) / self.max_vel
            new_amax = self.max_acc
        else:
            new_smaller_base = ((2 * delta_target) / self.max_vel) - target_time
            T_pico = (target_time - new_smaller_base) / 2
            T_total = target_time
            new_amax = self.max_vel / T_pico

        trajectory = [pos_init]
        current_time = 0.0
        current_vel = 0.0

        while current_time < T_pico:
            current_vel += new_amax * self.ts
            current_vel = min(current_vel, self.max_vel)
            trajectory.append(current_vel * move_sign * self.ts + trajectory[-1])
            current_time += self.ts

        while current_time < T_total - T_pico:
            trajectory.append(self.max_vel * move_sign * self.ts + trajectory[-1])
            current_time += self.ts

        while current_time < T_total:
            current_vel -= new_amax * self.ts
            trajectory.append(current_vel * move_sign * self.ts + trajectory[-1])
            current_time += self.ts

        return trajectory, T_total

    def single_traj(self, pos_init, pos_final, target_time=None):
        T_vmax = (self.max_vel) / (self.max_acc)
        delta_S = (self.max_vel**2) / self.max_acc
        delta_target = abs(pos_final - pos_init)

        if delta_S >= delta_target:
            trajectory, T_total = self.triangle_traj(pos_init, pos_final)
        else:
            trajectory, T_total = self.trapezoid_traj(pos_init, pos_final)

        if target_time is None:
            return trajectory, T_total

        if T_total >= (2 * delta_target) / self.max_vel:
            return self.triangle_traj(pos_init, pos_final, target_time=target_time)
        else:
            return self.trapezoid_traj(pos_init, pos_final, target_time=target_time)

    def generate_joint_trajectory(
        self, theta1_init, theta2_init, theta1_final, theta2_final
    ):
        delta1 = abs(theta1_final - theta1_init)
        delta2 = abs(theta2_final - theta2_init)

        if delta1 > delta2:
            biggest_traj, total_time = self.single_traj(theta1_init, theta1_final)
            smallest_traj, _ = self.single_traj(
                theta2_init, theta2_final, target_time=total_time
            )
            return [biggest_traj, smallest_traj]
        else:
            biggest_traj, total_time = self.single_traj(theta2_init, theta2_final)
            smallest_traj, _ = self.single_traj(
                theta1_init, theta1_final, target_time=total_time
            )
            return [smallest_traj, biggest_traj]
