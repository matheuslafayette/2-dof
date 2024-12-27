from transforms import SE2Transform
from trajectory import TrajectoryGenerator
import math

class RobotArm:
    def __init__(self, a1, a2, max_vel, max_acc, timestamp):
        self.a1 = a1
        self.a2 = a2
        self.trajectory_generator = TrajectoryGenerator(max_vel, max_acc, timestamp)

    def fk(self, theta1, theta2):
        H = (
            SE2Transform.theta(theta1)
            @ SE2Transform.xy(self.a1, 0)
            @ SE2Transform.theta(theta2)
            @ SE2Transform.xy(self.a2, 0)
        )
        x, y = H[:2, 2]
        theta = math.acos(H[0, 0])
        return x, y, theta

    def ik(self, x, y):
        solutions = []
        if math.hypot(x, y) > (self.a1 + self.a2) or math.hypot(x, y) < abs(
            self.a1 - self.a2
        ):
            return solutions

        cos_theta2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        if abs(cos_theta2) > 1:
            return solutions

        theta2 = math.acos(cos_theta2)
        k1 = self.a1 + self.a2 * math.cos(theta2)
        k2 = self.a2 * math.sin(theta2)

        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        solutions.append([theta1, theta2])

        if math.hypot(x, y) == (self.a1 + self.a2) or math.hypot(x, y) == abs(
            self.a1 - self.a2
        ):
            return solutions

        theta2_alt = -theta2
        k2_alt = self.a2 * math.sin(theta2_alt)
        theta1_alt = math.atan2(y, x) - math.atan2(k2_alt, k1)
        solutions.append([theta1_alt, theta2_alt])

        return solutions

    def proj_into_workspace(self, x, y):
        if math.hypot(x, y) <= (self.a1 + self.a2) and math.hypot(x, y) >= abs(
            self.a1 - self.a2
        ):
            return x, y

        theta = math.atan2(y, x)
        new_r = (
            self.a1 + self.a2
            if math.hypot(x, y) > (self.a1 + self.a2)
            else abs(self.a1 - self.a2)
        )

        return new_r * math.cos(theta), new_r * math.sin(theta)

    def traj_joint(self, theta1_init, theta2_init, theta1_final, theta2_final):
        return self.trajectory_generator.generate_joint_trajectory(
            theta1_init, theta2_init, theta1_final, theta2_final
        )

    def traj_eucl(self, x_init, y_init, x_final, y_final):
        x_init, y_init = self.proj_into_workspace(x_init, y_init)
        x_final, y_final = self.proj_into_workspace(x_final, y_final)

        delta_x = abs(x_final - x_init)
        delta_y = abs(y_final - y_init)

        if delta_x > delta_y:
            biggest_traj, total_time = self.trajectory_generator.single_traj(
                x_init, x_final
            )
            smallest_traj, _ = self.trajectory_generator.single_traj(
                y_init, y_final, target_time=total_time
            )
            traj_euclidean = [biggest_traj, smallest_traj]
        else:
            biggest_traj, total_time = self.trajectory_generator.single_traj(
                y_init, y_final
            )
            smallest_traj, _ = self.trajectory_generator.single_traj(
                x_init, x_final, target_time=total_time
            )
            traj_euclidean = [smallest_traj, biggest_traj]

        traj_joint = []
        for i in range(len(traj_euclidean[0])):
            xi, yi = self.proj_into_workspace(
                traj_euclidean[0][i], traj_euclidean[1][i]
            )
            ik_solutions = self.ik(xi, yi)
            if ik_solutions:
                theta1, theta2 = ik_solutions[0]
                traj_joint.append([theta1, theta2])

        return traj_joint
