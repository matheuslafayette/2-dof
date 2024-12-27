import numpy as np
import math
import matplotlib.pyplot as plt
#import roboticstoolbox as rtb

class SE2Transform:
    @staticmethod
    def xy(x, y):
        return np.array([
            [1, 0, x],
            [0, 1, y],
            [0, 0, 1]
        ])

    @staticmethod
    def theta(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

class RobotArm:
    def __init__(self, a1, a2, max_vel, max_acc, timestamp):
        self.a1 = a1
        self.a2 = a2
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.ts = timestamp

    def fk(self, theta1, theta2):
        H = (SE2Transform.theta(theta1) @ 
             SE2Transform.xy(self.a1, 0) @ 
             SE2Transform.theta(theta2) @ 
             SE2Transform.xy(self.a2, 0))
        x, y = H[:2, 2]
        theta = math.acos(H[0, 0])
        return x, y, theta

    def ik(self, x, y):
        solutions = []
        if math.hypot(x, y) > (self.a1 + self.a2) or math.hypot(x, y) < abs(self.a1 - self.a2):
            return solutions

        cos_theta2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        if abs(cos_theta2) > 1:
            return solutions

        theta2 = math.acos(cos_theta2)
        k1 = self.a1 + self.a2 * math.cos(theta2)
        k2 = self.a2 * math.sin(theta2)

        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        solutions.append([theta1, theta2])

        if math.hypot(x, y) == (self.a1 + self.a2) or math.hypot(x, y) == abs(self.a1 - self.a2):
            return solutions

        theta2_alt = -theta2
        k2_alt = self.a2 * math.sin(theta2_alt)
        theta1_alt = math.atan2(y, x) - math.atan2(k2_alt, k1)
        solutions.append([theta1_alt, theta2_alt])

        return solutions

    def triangle_traj(self, theta_init, theta_final, target_time=None):
        delta_target = abs(theta_final - theta_init)
        move_sign = 1 if theta_final >= theta_init else -1

        if target_time is None:
            T_pico = math.sqrt(delta_target/self.max_acc)
            new_amax = self.max_acc
        else:
            new_vmax = (2 * delta_target) / target_time
            new_amax = 2 * new_vmax / target_time
            T_pico = math.sqrt(delta_target/new_amax)

        trajectory = [theta_init]

        # Acceleration
        current_time = 0.0
        current_vel = 0.0

        while(current_time < T_pico):
            current_vel += new_amax*self.ts
            current_vel = min(current_vel, self.max_vel)
            trajectory.append(current_vel*move_sign*self.ts + trajectory[-1])
            current_time += self.ts
        
        # Deacceleration
        while(current_time < 2*T_pico):
            current_vel -= new_amax*self.ts
            trajectory.append(current_vel*move_sign*self.ts + trajectory[-1])
            current_time += self.ts

        return trajectory, 2 * T_pico
    
    def trapezoid_traj(self, theta_init, theta_final, target_time=None):
        delta_target = abs(theta_final - theta_init)
        move_sign = 1 if theta_final >= theta_init else -1

        if target_time == None:
            T_pico = self.max_vel/self.max_acc
            T_total = (delta_target + self.max_vel*T_pico)/self.max_vel
            new_amax = self.max_acc
        else:
            new_smaller_base = ((2 * delta_target) / self.max_vel) - target_time
            T_pico = (target_time - new_smaller_base) / 2 
            T_total = target_time
            new_amax = self.max_vel / T_pico

        trajectory = [theta_init]

        # Acceleration
        current_time = 0.0
        current_vel = 0.0
        while(current_time < T_pico):
            current_vel += new_amax*self.ts
            current_vel = min(current_vel, self.max_vel)
            trajectory.append(current_vel*move_sign*self.ts + trajectory[-1])
            current_time += self.ts

        # Constant vel
        while(current_time < T_total - T_pico):
            trajectory.append(self.max_vel*move_sign*self.ts + trajectory[-1])
            current_time += self.ts
        
        # Deacceleration
        while(current_time < T_total):
            current_vel -= new_amax*self.ts
            trajectory.append(current_vel*move_sign*self.ts + trajectory[-1])
            current_time += self.ts

        return trajectory, T_total

    def single_traj(self, theta_init, theta_final, target_time=None):

        T_vmax = (self.max_vel)/(self.max_acc) # time to reach max vel
        delta_S = (self.max_vel ** 2)/self.max_acc
        delta_target = abs(theta_final - theta_init)

        # Choose trajectory
        trajectory = []
        if (delta_S >= delta_target):
            trajectory, T_total = self.triangle_traj(theta_init, theta_final) 
            # traj_type = "Triangle"
        else:
            # traj_type = "Trapezoid"
            trajectory, T_total = self.trapezoid_traj(theta_init, theta_final)

        if target_time is None:
            return trajectory, T_total
        
        # Choose between triangle and trapezoid based on target Time
        if T_total >= (2 * delta_target) / self.max_vel:
            trajectory, T_total = self.triangle_traj(theta_init, theta_final, target_time=target_time)
            # traj_type = "Triangle"
            return trajectory, T_total
        else:
            trajectory, T_total = self.trapezoid_traj(theta_init, theta_final, target_time=target_time)
            # traj_type = "Trapezoid"
            return trajectory, T_total

    def traj_joint(self, theta1_init, theta2_init, theta1_final, theta2_final):
        delta1 = abs(theta1_final - theta1_init)
        delta2 = abs(theta2_final - theta2_init)

        biggest_delta = 1 if delta1 > delta2 else 2

        if (biggest_delta == 1):
            biggest_traj, total_time = self.single_traj(theta1_init, theta1_final)
            smallest_traj, total_time = self.single_traj(theta2_init, theta2_final, target_time=total_time)

            return [biggest_traj, smallest_traj]
        else:         

            biggest_traj, total_time = self.single_traj(theta2_init, theta2_final)
            smallest_traj, total_time = self.single_traj(theta1_init, theta1_final, target_time=total_time)

            return [smallest_traj, biggest_traj]
    
    def proj_into_workspace(self, x, y):
        if math.hypot(x, y) <= (self.a1 + self.a2) and math.hypot(x, y) >= abs(self.a1 - self.a2):
            return x, y
        
        #convert to polar coordinates
        theta = math.atan2(y, x)

        if math.hypot(x, y) > (self.a1 + self.a2):
            new_r = self.a1 + self.a2
        if math.hypot(x, y) < abs(self.a1 - self.a2):
            new_r = abs(self.a1 - self.a2)
        
        #convert back into cartesian coordinates
        new_x = new_r*math.cos(theta)
        new_y = new_r*math.sin(theta)
        return new_x, new_y
 
    def traj_eucl(self, x_init, y_init, x_final, y_final):
        x_init, y_init = self.proj_into_workspace(x_init, y_init)
        x_final, y_final = self.proj_into_workspace(x_final, y_final)
        
        delta_x = abs(x_final - x_init)
        delta_y = abs(y_final - y_init)

        biggest_delta = 1 if delta_x > delta_y else 2

        traj_euclidean = []

        if biggest_delta == 1:
            biggest_traj, total_time = self.single_traj(x_init, x_final)
            smallest_traj, total_time = self.single_traj(y_init, y_final, target_time=total_time)

            traj_euclidean = [biggest_traj, smallest_traj]
        else:
            biggest_traj, total_time = self.single_traj(y_init, y_final)
            smallest_traj, total_time = self.single_traj(x_init, x_final, target_time=total_time)

            traj_euclidean = [smallest_traj, biggest_traj]
        
        traj_joint = []

        # Convert to joint angles
        for i in range(len(traj_euclidean[0])):
            xi, yi = self.proj_into_workspace(traj_euclidean[0][i], traj_euclidean[1][i])
            ik_solutions = self.ik(xi, yi)
            if ik_solutions:
                theta1, theta2 = ik_solutions[0]
                traj_joint.append([theta1, theta2])

        return traj_joint

# Configuração do robô e visualização
def visualize_trajectory(trajectory, filename='RR.gif'):
    # Criar o robô usando a convenção DH da toolbox
    robot = rtb.models.DH.Planar2()

    # Gerar as trajetórias das juntas
    joint_trajectories = trajectory

    # Combinar as trajetórias das juntas em uma matriz para visualização
    nb_steps = len(joint_trajectories)
    print(nb_steps)
    qt = np.zeros((nb_steps, 2))
    for i in range(nb_steps):
        qt[i, :] = [joint_trajectories[i][0], joint_trajectories[i][1]]

    print(qt)

    # Visualizar a trajetória no espaço das juntas
    robot.plot(qt, backend='pyplot', movie=filename)
    print(f'Trajetória salva em: {filename}')

def plot(trajectory, ts, filename):
    time_steps = [i * ts for i in range(len(trajectory))]

    # vel
    velocity = [(trajectory[i + 1] - trajectory[i]) / ts for i in range(len(trajectory) - 1)]
    time_steps_vel = time_steps[:-1]

    # acc
    acceleration = [(velocity[i + 1] - velocity[i]) / ts for i in range(len(velocity) - 1)]
    time_steps_acc = time_steps_vel[:-1]

    # Fig 1: Traj plot
    plt.figure(figsize=(8, 5))
    plt.plot(time_steps, trajectory, marker='o', linestyle='-', linewidth=2, markersize=6)
    plt.title("Trajetória Gerada")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Posição")
    plt.grid(True)
    plt.savefig(f'{filename}.png')
    plt.close()

    # Fig 2: Vel plot
    plt.figure(figsize=(8, 5))
    plt.plot(time_steps_vel, velocity, marker='x', linestyle='-', color='red', linewidth=2)
    plt.title("Velocidade Derivada")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Velocidade")
    plt.grid(True)
    plt.savefig(f'{filename}_vel.png')
    plt.close()

    # Fig 3: Acc plot
    plt.figure(figsize=(8, 5))
    plt.plot(time_steps_acc, acceleration, marker='s', linestyle='-', color='blue', linewidth=2)
    plt.title("Aceleração Derivada")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Aceleração")
    plt.grid(True)
    plt.savefig(f'{filename}_acc.png')
    plt.close()

def main():
    arm = RobotArm(a1=1, a2=1, max_vel=2, max_acc=0.75, timestamp=0.01)
    
    # print("--------- Tests part 1 ---------")
    # p = np.array([0.5, 0.5, 1])
    # print("Test 1: ", (SE2Transform.xy(1, 0.25) @ p)[:2])
    # print("Test 2: ", (SE2Transform.xy(-1, -0.25) @ p)[:2])
    # print("Test 3: ", (SE2Transform.xy(1, 0.25) @ SE2Transform.theta(math.radians(45)) @ p)[:2])
    # print("Test 4: ", (np.linalg.inv(SE2Transform.xy(1, 0.25) @ SE2Transform.theta(math.radians(45))) @ p)[:2])
    
    # print("--------- Tests: FK ---------")
    # print("Test 1: ", arm.fk(0, math.pi/2))
    # print("Test 2: ", arm.fk(math.pi/2, math.pi/2))
    # print("Test 3: ", arm.fk(math.pi/2, -math.pi/2))
    # print("Test 4: ", arm.fk(-math.pi, math.pi))
    
    # print("--------- Tests: IK ---------")
    # print("Test 1: ", arm.ik(1, 1))
    # print("Test 2: ", arm.ik(1, -1))
    # print("Test 3: ", arm.ik(-1, 1))
    # print("Test 4: ", arm.ik(-1, -1))
    # print("Test 5: ", arm.ik(2, 1))
    # print("Test 6: ", arm.ik(2, 0))
    # print("Test 7: ", arm.ik(0, 2))
    # print("Test 8: ", arm.ik(-2, 0))

    # trajectory_trapezoid = arm.traj_joint(2, 5, 0.5, 1)
    # # print(trajectory_trapezoid)
    # # trajectory_triangle = arm.traj_joint(0.0, 1.0, 0.0, 1.0)
    # # print(trajectory_triangle)

    # plot(trajectory_trapezoid[0], arm.ts, "traj_joint1")
    # plot(trajectory_trapezoid[1], arm.ts, "traj_joint2")

    print("--------- Tests: traj_eucl ---------")
    trajectory_eucl = arm.traj_eucl(0, 1, 1, 0)
    # print(trajectory_eucl)

    #visualize_trajectory(trajectory_eucl, filename='RR.gif')

if __name__ == '__main__':
    main()